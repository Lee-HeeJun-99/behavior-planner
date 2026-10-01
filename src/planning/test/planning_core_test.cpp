#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <gtest/gtest.h>
#include "planning_pkg_2025/behavior/behavior_planner.hpp"
#include "planning_pkg_2025/context/context_manager.hpp"
#include "planning_pkg_2025/local/local_planner.hpp"
#include "planning_pkg_2025/map/global_path.hpp"
namespace planning {
Path straightPath() {Path p; for (int i=0;i<300;++i) p.push_back({i*0.1,0,0,0}); return p;}
TEST(BehaviorPlanner, ConfirmsAndClearsAvoidance) {
  auto path=straightPath(); BehaviorContext c; c.vehicle.localization_valid=true; c.path={&path,0};
  c.obstacles={{{3,0},0.3}}; BehaviorConfig cfg; cfg.detection_confirmation_count=2;
  cfg.clear_confirmation_count=2; cfg.minimum_behavior_duration=std::chrono::milliseconds(0);
  BehaviorPlanner planner(cfg); auto now=std::chrono::steady_clock::now();
  EXPECT_EQ(planner.update(c,now),Behavior::CRUISE); EXPECT_EQ(planner.update(c,now),Behavior::AVOID);
  c.obstacles.clear(); EXPECT_EQ(planner.update(c,now),Behavior::AVOID); EXPECT_EQ(planner.update(c,now),Behavior::CRUISE);
}
ContextConfig speedConfig() {
  ContextConfig config; config.speed_sign_confirmation_count=3;
  config.speed_limit_30_target=2.5; config.speed_limit_40_target=3.5; config.speed_limit_50_target=4.5;
  return config;
}
TEST(SpeedContext, StartsAtDefaultSpeed) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  const auto context=manager.build(path,0);
  EXPECT_EQ(context.speed.active_limit_kph,30); EXPECT_DOUBLE_EQ(context.speed.target_velocity,2.5);
  EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::CRUISE,context),2.5);
}
TEST(SpeedContext, ConfirmsThirtySign) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(30); manager.updateSpeedLimitDetection(30); manager.updateSpeedLimitDetection(30);
  const auto context=manager.build(path,0);
  EXPECT_TRUE(context.speed.detection_valid); EXPECT_EQ(context.speed.detected_limit_kph,30);
  EXPECT_EQ(context.speed.active_limit_kph,30);
}
TEST(SpeedContext, IgnoresSingleFalseFiftyDetection) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50);
  EXPECT_EQ(manager.build(path,0).speed.active_limit_kph,30);
}
TEST(SpeedContext, InterruptedDetectionsDoNotConfirm) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(0);
  manager.updateSpeedLimitDetection(50);
  EXPECT_EQ(manager.build(path,0).speed.active_limit_kph,30);
}
TEST(SpeedContext, ActivatesConfirmedFiftyLimit) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50);
  const auto context=manager.build(path,0);
  EXPECT_EQ(context.speed.active_limit_kph,50); EXPECT_DOUBLE_EQ(context.speed.target_velocity,4.5);
}
TEST(SpeedContext, KeepsActiveLimitWhenDetectionDisappears) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50);
  manager.updateSpeedLimitDetection(0); const auto context=manager.build(path,0);
  EXPECT_FALSE(context.speed.detection_valid); EXPECT_EQ(context.speed.active_limit_kph,50);
  EXPECT_DOUBLE_EQ(context.speed.target_velocity,4.5);
}
TEST(SpeedContext, RejectsUnsupportedDetection) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(70); manager.updateSpeedLimitDetection(70);
  manager.updateSpeedLimitDetection(70);
  const auto context=manager.build(path,0);
  EXPECT_FALSE(context.speed.detection_valid); EXPECT_EQ(context.speed.active_limit_kph,30);
}
TEST(SpeedContext, InvalidDefaultFallsBackConsistently) {
  auto path=straightPath(); auto config=speedConfig(); config.default_speed_limit_kph=70;
  ContextManager manager(config); manager.updateVehicle({0,0},0);
  const auto context=manager.build(path,0);
  EXPECT_EQ(context.speed.active_limit_kph,30);
  EXPECT_DOUBLE_EQ(context.speed.target_velocity,2.5);
}
TEST(SpeedContext, AvoidancePreservesActiveLimit) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50);
  manager.updateObstacles({{8,0}}); auto context=manager.build(path,0); context.free_space={2.5,2.5};
  BehaviorConfig behavior_config; behavior_config.detection_confirmation_count=1;
  behavior_config.minimum_behavior_duration=std::chrono::milliseconds(0);
  BehaviorPlanner behavior_planner(behavior_config); const auto now=std::chrono::steady_clock::now();
  EXPECT_EQ(behavior_planner.update(context,now),Behavior::AVOID);
  PathGeneratorConfig generator; generator.lateral_offsets={-1.5,0,1.5};
  auto plan=LocalPlanner(PathGenerator(generator),CollisionChecker{},CostEvaluator{}).plan(Behavior::AVOID,context);
  EXPECT_TRUE(plan.feasible); EXPECT_EQ(context.speed.active_limit_kph,50);
  EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::AVOID,context),2.5);
}
TEST(SpeedContext, EmergencyStopPreservesLimitAndResumesIt) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50);
  BehaviorPlanner planner; const auto now=std::chrono::steady_clock::now();
  auto context=manager.build(path,0); EXPECT_EQ(planner.update(context,now),Behavior::CRUISE);
  manager.setEmergencyStop(true); context=manager.build(path,0);
  EXPECT_EQ(planner.update(context,now),Behavior::EMERGENCY_STOP);
  EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::EMERGENCY_STOP,context),0.0);
  EXPECT_EQ(context.speed.active_limit_kph,50);
  manager.setEmergencyStop(false); context=manager.build(path,0);
  EXPECT_EQ(planner.update(context,now),Behavior::CRUISE);
  EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::CRUISE,context),4.5);
}
TEST(LocalPlanner, SelectsCollisionFreeOffsetAndRecovers) {
  auto path=straightPath(); BehaviorContext c; c.vehicle.localization_valid=true; c.path={&path,0};
  c.free_space={2,2}; c.obstacles={{{8,0},0.25}}; PathGeneratorConfig gen;
  gen.lateral_offsets={-1,0,1}; gen.horizon_points=250; gen.transition_points=60;
  CollisionConfig col; col.vehicle_half_width=0.3; col.vehicle_front=0.5; col.vehicle_rear=0.3; col.safety_margin=0.1;
  LocalPlanner planner(PathGenerator(gen),CollisionChecker(col),CostEvaluator{});
  auto avoid=planner.plan(Behavior::AVOID,c); ASSERT_TRUE(avoid.feasible); EXPECT_NE(avoid.selected_offset,0);
  c.obstacles.clear(); auto cruise=planner.plan(Behavior::CRUISE,c); ASSERT_TRUE(cruise.feasible); EXPECT_DOUBLE_EQ(cruise.selected_offset,0);
}
TEST(LocalPlanner, RejectsCandidatesOutsideBoundary) {
  auto path=straightPath(); BehaviorContext c; c.vehicle.localization_valid=true; c.path={&path,0}; c.free_space={0.2,0.2};
  EXPECT_FALSE(LocalPlanner{}.plan(Behavior::AVOID,c).feasible);
}
TEST(LocalPlanner, RejectsAllCandidatesWhenCorridorIsBlocked) {
  auto path=straightPath(); BehaviorContext c; c.vehicle.localization_valid=true; c.path={&path,0};
  c.free_space={2.5,2.5};
  for (double lateral=-2.0; lateral<=2.0; lateral+=0.5) c.obstacles.push_back({{8,lateral},0.35});
  PathGeneratorConfig gen; gen.lateral_offsets={-1.5,-1.2,-0.9,-0.6,-0.3,0,0.3,0.6,0.9,1.2,1.5};
  auto result=LocalPlanner(PathGenerator(gen),CollisionChecker{},CostEvaluator{}).plan(Behavior::AVOID,c);
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.candidates.size(),11u);
  EXPECT_EQ(std::count_if(result.candidates.begin(),result.candidates.end(),
    [](const CandidatePath & candidate) {return candidate.valid;}),0);
}
TEST(LocalPlanner, ReportsProductionGeometryForCentralObstacle) {
  auto path=straightPath(); BehaviorContext c; c.vehicle.localization_valid=true; c.path={&path,0};
  c.free_space={2.5,2.5}; c.obstacles={{{8,0},0.35}};
  PathGeneratorConfig gen; gen.lateral_offsets={-1.5,-1.2,-0.9,-0.6,-0.3,0,0.3,0.6,0.9,1.2,1.5};
  auto result=LocalPlanner(PathGenerator(gen),CollisionChecker{},CostEvaluator{}).plan(Behavior::AVOID,c);
  for (const auto & candidate : result.candidates) {
    std::cout << "offset=" << candidate.lateral_offset << " valid=" << candidate.valid
              << " reason=" << candidate.invalid_reason << " clearance=" << candidate.minimum_clearance
              << " cost=" << candidate.cost << '\n';
  }
  EXPECT_TRUE(result.feasible);
}
TEST(LocalPlanner, ReportsSurveyedMapGeometryWhenRequested) {
  const char * filename=std::getenv("PLANNING_TEST_MAP");
  if (!filename) GTEST_SKIP() << "PLANNING_TEST_MAP is unset";
  GlobalPath global; std::string error; ASSERT_TRUE(global.load(filename,&error)) << error;
  const auto & path=global.path(); ASSERT_GT(path.size(),160u);
  BehaviorContext c; c.vehicle.localization_valid=true; c.path={&path,0}; c.free_space={2.5,2.5};
  c.obstacles={{{path[160].x,path[160].y},0.35}};
  PathGeneratorConfig gen; gen.lateral_offsets={-1.5,-1.2,-0.9,-0.6,-0.3,0,0.3,0.6,0.9,1.2,1.5}; gen.transition_points=120;
  auto result=LocalPlanner(PathGenerator(gen),CollisionChecker{},CostEvaluator{}).plan(Behavior::AVOID,c);
  for (const auto & candidate : result.candidates) {
    std::cout << "offset=" << candidate.lateral_offset << " valid=" << candidate.valid
              << " reason=" << candidate.invalid_reason << " clearance=" << candidate.minimum_clearance
              << " cost=" << candidate.cost << '\n';
  }
  EXPECT_TRUE(result.feasible);
}
}
