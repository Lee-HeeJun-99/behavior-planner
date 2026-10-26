#include <algorithm>
#include "planning_pkg_2025/behavior/auto_entry_guard.hpp"
#include "planning_pkg_2025/behavior/recovery_guard.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cstdio>
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
  config.speed_limit_20_target=2.5; config.speed_limit_50_target=4.5;
  return config;
}
TEST(SpeedContext, StartsAtDefaultSpeed) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  const auto context=manager.build(path,0);
  EXPECT_EQ(context.speed.active_limit_kph,20); EXPECT_DOUBLE_EQ(context.speed.target_velocity,2.5);
  EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::CRUISE,context),2.5);
}
TEST(SpeedContext, ConfirmsTwentySign) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50);
  manager.updateSpeedLimitDetection(20); manager.updateSpeedLimitDetection(20); manager.updateSpeedLimitDetection(20);
  const auto context=manager.build(path,0);
  EXPECT_TRUE(context.speed.detection_valid); EXPECT_EQ(context.speed.detected_limit_kph,20);
  EXPECT_EQ(context.speed.active_limit_kph,20); EXPECT_DOUBLE_EQ(context.speed.target_velocity,2.5);
}
TEST(SpeedContext, IgnoresSingleFalseFiftyDetection) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50);
  EXPECT_EQ(manager.build(path,0).speed.active_limit_kph,20);
}
TEST(SpeedContext, InterruptedDetectionsDoNotConfirm) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(0);
  manager.updateSpeedLimitDetection(50);
  EXPECT_EQ(manager.build(path,0).speed.active_limit_kph,20);
}
TEST(SpeedContext, DifferentSupportedSignResetsCandidate) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(50); manager.updateSpeedLimitDetection(50);
  manager.updateSpeedLimitDetection(20); manager.updateSpeedLimitDetection(50);
  EXPECT_EQ(manager.build(path,0).speed.active_limit_kph,20);
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
  manager.updateSpeedLimitDetection(0); manager.updateSpeedLimitDetection(0);
  manager.updateSpeedLimitDetection(0); const auto context=manager.build(path,0);
  EXPECT_FALSE(context.speed.detection_valid); EXPECT_EQ(context.speed.active_limit_kph,50);
  EXPECT_DOUBLE_EQ(context.speed.target_velocity,4.5);
}
TEST(SpeedContext, RejectsUnsupportedDetection) {
  auto path=straightPath(); ContextManager manager(speedConfig()); manager.updateVehicle({0,0},0);
  manager.updateSpeedLimitDetection(30); manager.updateSpeedLimitDetection(40);
  manager.updateSpeedLimitDetection(70);
  const auto context=manager.build(path,0);
  EXPECT_FALSE(context.speed.detection_valid); EXPECT_EQ(context.speed.active_limit_kph,20);
}
TEST(SpeedContext, InvalidDefaultFallsBackConsistently) {
  auto path=straightPath(); auto config=speedConfig(); config.default_speed_limit_kph=70;
  ContextManager manager(config); manager.updateVehicle({0,0},0);
  const auto context=manager.build(path,0);
  EXPECT_EQ(context.speed.active_limit_kph,20);
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
  EXPECT_GE(result.candidates.size(),11u);
  const auto valid_count = std::count_if(
    result.candidates.begin(), result.candidates.end(),
    [](const CandidatePath & candidate) {return candidate.valid;});
  EXPECT_EQ(valid_count, 0);
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
TEST(DistanceAvoidance, ApproachesBeforeActivationAndHoldsUntilRearClearance) {
  auto path=straightPath(); BehaviorContext c; c.path={&path,0};
  c.vehicle.localization_valid=true; c.free_space={3,3}; c.obstacles={{{13,0},0.35}};
  BehaviorConfig b; b.detection_confirmation_count=1; b.clear_confirmation_count=1;
  b.minimum_behavior_duration=std::chrono::milliseconds(0);
  BehaviorPlanner behavior(b); auto now=std::chrono::steady_clock::now();
  EXPECT_EQ(behavior.update(c,now),Behavior::CRUISE);
  PathGeneratorConfig g;g.lateral_offsets={-1.8,-1.5,0,1.5,1.8};g.transition_points=60;
  LocalPlanner planner(PathGenerator(g),CollisionChecker{},CostEvaluator{});
  auto approach=planner.plan(Behavior::CRUISE,c);
  ASSERT_TRUE(approach.feasible);EXPECT_LT(approach.path.back().x,10.1);
  c.path.nearest_index=40;c.vehicle.position={4,0};
  EXPECT_EQ(behavior.update(c,now),Behavior::AVOID);
  c.path.nearest_index=140;c.vehicle.position={14,1.5};
  EXPECT_EQ(behavior.update(c,now),Behavior::AVOID);
  c.path.nearest_index=170;c.vehicle.position={17,1.5};
  EXPECT_EQ(behavior.update(c,now),Behavior::AVOID);
  c.vehicle.position={17,.1};
  EXPECT_EQ(behavior.update(c,now),Behavior::CRUISE);
}
TEST(DistanceAvoidance, DoesNotReturnBeforeFarObstacle) {
  auto path=straightPath();BehaviorContext c;c.path={&path,0};c.vehicle.localization_valid=true;
  c.free_space={3,3};c.obstacles={{{13,0},0.35}};
  PathGeneratorConfig g;g.lateral_offsets={-1.8,-1.5,0,1.5,1.8};g.transition_points=60;
  auto plan=LocalPlanner(PathGenerator(g),CollisionChecker{},CostEvaluator{}).plan(Behavior::AVOID,c);
  ASSERT_TRUE(plan.feasible);EXPECT_GE(std::abs(plan.path[130].y),1.5);
}
TEST(DistanceAvoidance, CloseObstacleStillStopsWhenNoCollisionFreePathExists) {
  auto path=straightPath();BehaviorContext c;c.path={&path,0};c.vehicle.localization_valid=true;
  c.free_space={3,3};c.obstacles={{{1,0},0.35}};
  PathGeneratorConfig g;g.lateral_offsets={-1.8,0,1.8};
  EXPECT_FALSE(LocalPlanner(PathGenerator(g),CollisionChecker{},CostEvaluator{}).plan(Behavior::AVOID,c).feasible);
}

TEST(DistanceAvoidance, RechecksCommittedPathForNewCollision) {
 auto path=straightPath();BehaviorContext c;c.path={&path,0};c.vehicle.localization_valid=true;
 c.free_space={3,3};c.obstacles={{{8,0},0.35}};
 PathGeneratorConfig g;g.lateral_offsets={-1.8,-1.5,0,1.5,1.8};g.transition_points=60;
 LocalPlanner planner{PathGenerator(g),CollisionChecker{},CostEvaluator{}};
 auto plan=planner.plan(Behavior::AVOID,c);ASSERT_TRUE(plan.feasible);
 c.vehicle.position={plan.path[5].x,plan.path[5].y};
 c.obstacles.push_back({c.vehicle.position,0.35});
 EXPECT_FALSE(planner.plan(Behavior::AVOID,c).feasible);
}
TEST(DistanceAvoidance, ContinuousActualMapApproachPassAndReturn) {
 const char * filename=std::getenv("PLANNING_TEST_MAP");ASSERT_NE(filename,nullptr);
 GlobalPath global;std::string error;ASSERT_TRUE(global.load(filename,&error));const auto &path=global.path();
 ContextConfig cc;cc.road_left_bound=3;cc.road_right_bound=3;ContextManager manager(cc);
 manager.updateObstacles({{path[260].x,path[260].y}});
 BehaviorPlanner behavior;PathGeneratorConfig g;g.transition_points=120;
 g.lateral_offsets={-1.8,-1.5,-1.2,-.9,-.6,-.3,0,.3,.6,.9,1.2,1.5,1.8};
 LocalPlanner planner{PathGenerator(g),CollisionChecker{},CostEvaluator{}};
 Point2d position{path[0].x,path[0].y};double yaw=path[0].yaw;size_t index=0;bool returned=false;
 auto time=std::chrono::steady_clock::now();
 for(int step=0;step<120;++step){
  index=global.nearestIndex(position,index);manager.updateVehicle(position,yaw);auto c=manager.build(path,index);
  auto b=behavior.update(c,time+std::chrono::milliseconds(step*100));auto plan=planner.plan(b,c);
  if(!plan.feasible&&b==Behavior::CRUISE&&!c.obstacles.empty())plan=planner.plan(Behavior::AVOID,c);
  ASSERT_TRUE(plan.feasible)<<"step="<<step<<" index="<<index;
  ASSERT_GE(plan.path.size(),3u);const size_t advance=std::min(size_t(5),plan.path.size()-1);
  position={plan.path[advance].x,plan.path[advance].y};yaw=plan.path[advance].yaw;
  const double lateral=-std::sin(path[index].yaw)*(position.x-path[index].x)+std::cos(path[index].yaw)*(position.y-path[index].y);
  if(index>420&&std::abs(lateral)<.15&&b==Behavior::CRUISE){returned=true;break;}
 }
 EXPECT_TRUE(returned);
}

TEST(GlobalPathGeometry, SmoothsSurveyNoiseAndRecomputesStaleYaw) {
 const std::string filename="/tmp/planning_survey_noise_regression.csv";
 {std::ofstream f(filename);for(int i=0;i<500;++i)f<<i*.05<<","<<.01*std::sin(i*.5)<<",1.5,99\n";}
 GlobalPath raw,smoothed;std::string error;
 ASSERT_TRUE(raw.load(filename,&error,0.0))<<error;
 ASSERT_TRUE(smoothed.load(filename,&error,.75))<<error;
 std::remove(filename.c_str());
 BehaviorContext c;c.vehicle.localization_valid=true;c.free_space={3,3};c.path={&raw.path(),100};
 c.vehicle.position={raw.path()[100].x,raw.path()[100].y};
 EXPECT_FALSE(LocalPlanner{}.plan(Behavior::CRUISE,c).feasible);
 c.path={&smoothed.path(),100};c.vehicle.position={smoothed.path()[100].x,smoothed.path()[100].y};
 EXPECT_TRUE(LocalPlanner{}.plan(Behavior::CRUISE,c).feasible);
 EXPECT_LT(std::abs(smoothed.path()[100].yaw),.01);
 for(size_t j=0;j<raw.path().size();++j){
  EXPECT_LE(std::hypot(raw.path()[j].x-smoothed.path()[j].x,raw.path()[j].y-smoothed.path()[j].y),.300001);
 }
 EXPECT_DOUBLE_EQ(raw.path().front().x,smoothed.path().front().x);
 EXPECT_DOUBLE_EQ(raw.path().back().x,smoothed.path().back().x);
}

TEST(SpeedContext, OneKphDefaultAndConfirmedTwentyRaisesToTwoKph) {
 auto path=straightPath();ContextConfig config;config.default_speed_limit_kph=0;
 config.default_target_velocity=1.0/3.6;config.speed_limit_20_target=2.0/3.6;
 config.avoid_max_velocity=2.0/3.6;ContextManager manager(config);manager.updateVehicle({0,0},0);
 EXPECT_EQ(manager.build(path,0).speed.active_limit_kph,0);
 EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::CRUISE,manager.build(path,0)),1.0/3.6);
 manager.updateSpeedLimitDetection(20);manager.updateSpeedLimitDetection(20);
 EXPECT_DOUBLE_EQ(manager.build(path,0).speed.target_velocity,1.0/3.6);
 manager.updateSpeedLimitDetection(20);
 EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::CRUISE,manager.build(path,0)),2.0/3.6);
 manager.updateSpeedLimitDetection(0);
 EXPECT_DOUBLE_EQ(manager.build(path,0).speed.target_velocity,2.0/3.6);
 EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::AVOID,manager.build(path,0)),2.0/3.6);
 EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::EMERGENCY_STOP,manager.build(path,0)),0.0);
}

TEST(RecoveryGuard, CloseObstacleStopsThenRequiresStoppedStableFeasibleConfirmation) {
 BehaviorContext c;c.vehicle.localization_valid=true;c.obstacles={{{2,0},.35}};
 RecoveryGuard guard;auto now=std::chrono::steady_clock::now();
 EXPECT_EQ(guard.apply(c,Behavior::AVOID,true,false,true,now),Behavior::EMERGENCY_STOP);
 for(int i=1;i<=30;++i)EXPECT_EQ(guard.apply(c,Behavior::AVOID,true,false,true,now+std::chrono::milliseconds(i*50)),Behavior::EMERGENCY_STOP);
 for(int i=31;i<40;++i)EXPECT_EQ(guard.apply(c,Behavior::AVOID,true,true,true,now+std::chrono::milliseconds(i*50)),Behavior::EMERGENCY_STOP);
 EXPECT_EQ(guard.apply(c,Behavior::AVOID,true,true,true,now+std::chrono::milliseconds(2000)),Behavior::AVOID);
 EXPECT_TRUE(guard.lowSpeed());EXPECT_DOUBLE_EQ(guard.speedCap(),.5/3.6);
 EXPECT_EQ(guard.apply(c,Behavior::AVOID,true,false,true,now+std::chrono::milliseconds(2050)),Behavior::AVOID);
}
TEST(RecoveryGuard, ChangingObstacleSetsAndStaleInputCannotReleaseStop) {
 BehaviorContext c;c.vehicle.localization_valid=true;c.obstacles={{{2,0},.35}};
 RecoveryGuard guard;auto now=std::chrono::steady_clock::now();
 for(int i=0;i<100;++i){c.obstacles[0].center.y=i%2?.7:0;
 EXPECT_EQ(guard.apply(c,Behavior::AVOID,true,true,true,now+std::chrono::milliseconds(i*50)),Behavior::EMERGENCY_STOP);}
 for(int i=100;i<140;++i)EXPECT_EQ(guard.apply(c,Behavior::AVOID,true,true,false,now+std::chrono::milliseconds(i*50)),Behavior::EMERGENCY_STOP);
}
TEST(RecoveryGuard, AlternatingFeasiblePathsNeverReachRestartThreshold) {
 BehaviorContext c;c.vehicle.localization_valid=true;c.obstacles={{{9,0},.35}};
 RecoveryGuard guard;auto now=std::chrono::steady_clock::now();
 for(int i=0;i<100;++i)EXPECT_EQ(guard.apply(c,Behavior::AVOID,i%2==1,true,true,now+std::chrono::milliseconds(i*50)),Behavior::EMERGENCY_STOP);
}
TEST(RecoveryGuard, ExplicitEmergencyAndCollisionRemainStopped) {
 BehaviorContext c;c.vehicle.localization_valid=true;c.emergency_stop=true;
 RecoveryGuard guard;auto now=std::chrono::steady_clock::now();
 for(int i=0;i<100;++i)EXPECT_EQ(guard.apply(c,Behavior::CRUISE,true,true,true,now+std::chrono::milliseconds(i*50)),Behavior::EMERGENCY_STOP);
 c.emergency_stop=false;
 for(int i=100;i<120;++i)EXPECT_EQ(guard.apply(c,Behavior::AVOID,false,true,true,now+std::chrono::milliseconds(i*50)),Behavior::EMERGENCY_STOP);
}

}

namespace planning {
TEST(GenericObstacles, OffPathRemainInContextWithoutAvoidance) {
  auto path = straightPath();
  ContextManager manager(speedConfig());
  manager.updateVehicle({0, 0}, 0);
  manager.updateObstacles({{8, -3}, {8, -2.5}, {8, 2.5}, {8, 3}, {30, 0}});
  auto context = manager.build(path, 0);
  ASSERT_EQ(context.obstacles.size(), 4u);
  BehaviorPlanner behavior;
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(behavior.update(context, std::chrono::steady_clock::now()), Behavior::CRUISE);
  }
  EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::CRUISE, context), 2.5);
  LocalPlanner local;
  auto plan = local.plan(Behavior::CRUISE, context);
  EXPECT_TRUE(plan.feasible);
  EXPECT_DOUBLE_EQ(plan.selected_offset, 0.0);
}

TEST(GenericObstacles, SideObstacleInvalidatesCandidatesAndSelectedPathIsSafe) {
  auto path = straightPath();
  auto config = speedConfig();
  config.road_left_bound = config.road_right_bound = 2.5;
  ContextManager manager(config);
  manager.updateVehicle({0, 0}, 0);
  manager.updateObstacles({{8, 0}, {8, -1.8}});
  auto context = manager.build(path, 0);
  BehaviorPlanner behavior;
  for (int i = 0; i < 3; ++i) {
    behavior.update(context, std::chrono::steady_clock::now());
  }
  ASSERT_EQ(behavior.current(), Behavior::AVOID);
  PathGeneratorConfig generator_config;
  generator_config.lateral_offsets = {-1.5, -1.2, -0.9, 0.0, 0.9, 1.2, 1.5};
  LocalPlanner local{PathGenerator(generator_config), CollisionChecker{}, CostEvaluator{}};
  auto plan = local.plan(behavior.current(), context);
  ASSERT_TRUE(plan.feasible);
  bool side_collision = false;
  auto side_only = context;
  side_only.obstacles.erase(side_only.obstacles.begin());
  CollisionChecker checker;
  for (auto candidate : plan.candidates) {
    if (candidate.lateral_offset < 0 && !checker.validate(candidate, side_only) &&
      candidate.invalid_reason == "obstacle_collision") {side_collision = true;}
  }
  EXPECT_TRUE(side_collision);
  CandidatePath selected;
  selected.path = plan.path;
  selected.lateral_offset = plan.selected_offset;
  EXPECT_TRUE(checker.validate(selected, context));
}
}  // namespace planning

namespace planning {
TEST(GenericObstacles, SevenScenarioRegression) {
  const std::vector<std::pair<std::string, std::vector<Point2d>>> scenarios = {
    {"cruise", {}}, {"avoid_center", {{8, 0}}},
    {"avoid_left", {{8, 0.75}}}, {"avoid_right", {{8, -0.75}}},
    {"blocked", {{8, -2.5}, {8, -2}, {8, -1.5}, {8, -1}, {8, -0.5},
      {8, 0}, {8, 0.5}, {8, 1}, {8, 1.5}, {8, 2}, {8, 2.5}}},
    {"off_path_obstacles", {{8, -3}, {8, -2.5}, {8, 2.5}, {8, 3}}},
    {"avoid_with_side_obstacle", {{8, 0}, {8, -1.8}}}
  };
  auto path = straightPath();
  for (const auto & scenario : scenarios) {
    SCOPED_TRACE(scenario.first);
    auto config = speedConfig();
    config.road_left_bound = config.road_right_bound = 2.5;
    ContextManager manager(config);
    manager.updateVehicle({0, 0}, 0);
    manager.updateObstacles(scenario.second);
    auto context = manager.build(path, 0);
    ASSERT_EQ(context.obstacles.size(), scenario.second.size());
    BehaviorPlanner behavior;
    for (int i = 0; i < 3; ++i) {
      behavior.update(context, std::chrono::steady_clock::now());
    }
    const bool cruise = scenario.first == "cruise" || scenario.first == "off_path_obstacles";
    EXPECT_EQ(behavior.current(), cruise ? Behavior::CRUISE : Behavior::AVOID);
    PathGeneratorConfig generator;
    generator.lateral_offsets = {-1.5, -1.2, -0.9, -0.6, -0.3, 0, 0.3, 0.6, 0.9, 1.2, 1.5};
    LocalPlanner local{PathGenerator(generator), CollisionChecker{}, CostEvaluator{}};
    auto plan = local.plan(behavior.current(), context);
    if (scenario.first == "blocked") {
      EXPECT_FALSE(plan.feasible);
      EXPECT_TRUE(plan.path.empty());
      const auto valid = std::count_if(plan.candidates.begin(), plan.candidates.end(),
        [](const CandidatePath & candidate) {return candidate.valid;});
      EXPECT_EQ(valid, 0);
      // PlanningNode maps an infeasible LocalPlan to EMERGENCY_STOP.
      EXPECT_DOUBLE_EQ(calculateTargetVelocity(Behavior::EMERGENCY_STOP, context), 0.0);
    } else {
      EXPECT_TRUE(plan.feasible);
      if (cruise) {EXPECT_DOUBLE_EQ(plan.selected_offset, 0.0);}
    }
  }
}
}  // namespace planning

namespace planning {
TEST(RecoveryCorridor, DistantSceneChangesDoNotPreventConfirmedRestart) {
 BehaviorContext c;c.vehicle.localization_valid=true;c.obstacles={{{2,0},.35}};
 auto path=straightPath();RecoveryGuard guard;auto now=std::chrono::steady_clock::now();
 Behavior result=Behavior::CRUISE;
 for(int i=0;i<31;++i){
  c.obstacles={{{2,0},.35},{{15,3.+i},.35}};
  if(i%2)c.obstacles.push_back({{19,-3},.35});
  result=guard.apply(c,Behavior::AVOID,true,true,true,now+std::chrono::milliseconds(i*50),&path);
 }
 EXPECT_EQ(result,Behavior::AVOID);EXPECT_TRUE(guard.lowSpeed());EXPECT_EQ(guard.relevantCount(),1u);
}
TEST(RecoveryCorridor, TurningPathKeepsObstaclesOutsideStraightBodyCorridorRelevant) {
 BehaviorContext c;c.vehicle.localization_valid=true;
 Path path;for(int i=0;i<=60;++i)path.push_back({0,i*.1,M_PI/2,0});
 RecoveryGuard guard;auto now=std::chrono::steady_clock::now();
 c.obstacles={{{0,5},.35}};
 EXPECT_EQ(guard.apply(c,Behavior::AVOID,false,true,true,now-std::chrono::milliseconds(50),&path),Behavior::EMERGENCY_STOP);
 for(int i=0;i<100;++i){c.obstacles={{{i%2?.7:0,5},.35}};
 EXPECT_EQ(guard.apply(c,Behavior::AVOID,true,true,true,now+std::chrono::milliseconds(i*50),&path),Behavior::EMERGENCY_STOP);
 EXPECT_EQ(guard.relevantCount(),1u);}
}
TEST(RecoveryCorridor, IgnoredStabilityObstacleStillStopsWhenPathValidationFails) {
 BehaviorContext c;c.vehicle.localization_valid=true;c.obstacles={{{12,0},.35}};
 auto path=straightPath();RecoveryGuard guard;auto now=std::chrono::steady_clock::now();
 for(int i=0;i<100;++i)EXPECT_EQ(guard.apply(c,Behavior::AVOID,false,true,true,now+std::chrono::milliseconds(i*50),&path),Behavior::EMERGENCY_STOP);
 EXPECT_EQ(guard.relevantCount(),0u);
}
}

namespace planning {
TEST(RecoveryCorridor, ReturnsToNormalAfterClearCorridorDespiteDistantObstacles) {
 BehaviorContext c;c.vehicle.localization_valid=true;c.obstacles={{{15,4},.35}};
 auto path=straightPath();RecoveryGuard guard;auto now=std::chrono::steady_clock::now();
 EXPECT_EQ(guard.apply(c,Behavior::CRUISE,false,true,true,now,&path),Behavior::EMERGENCY_STOP);
 for(int i=1;i<=29;++i)guard.apply(c,Behavior::CRUISE,true,true,true,now+std::chrono::milliseconds(i*50),&path);
 EXPECT_FALSE(guard.holding());EXPECT_TRUE(guard.lowSpeed());
 guard.apply(c,Behavior::CRUISE,true,true,true,now+std::chrono::milliseconds(2000),&path);
 EXPECT_TRUE(guard.lowSpeed());
 guard.apply(c,Behavior::CRUISE,true,true,true,now+std::chrono::milliseconds(2500),&path);
 EXPECT_FALSE(guard.lowSpeed());EXPECT_EQ(c.obstacles.size(),1u);
}
}

namespace planning {
Path circleRoute() {
 Path p;const int n=800;for(int i=0;i<n;++i){double a=2*M_PI*i/n;p.push_back({8*std::cos(a),8*std::sin(a),a+M_PI/2,1./8});}return p;
}
TEST(ClosedLoop, IndexAndLocalHorizonWrapForThreeLaps) {
 GlobalPath global;global.setClosed(true);global.setPath(circleRoute());
 EXPECT_EQ(global.routeSize(),800u);EXPECT_GE(global.path().size(),1100u);
 PathGeneratorConfig config;LocalPlanner planner{PathGenerator(config),CollisionChecker{},CostEvaluator{}};
 std::size_t hint=global.routeSize()-20;int wraps=0;
 for(std::size_t progress=global.routeSize()-20;progress<global.routeSize()*4-20;progress+=5){
  auto expected=progress%global.routeSize();const auto &p=global.path()[expected];
  const auto index=global.nearestIndex({p.x,p.y},hint);if(index<hint)++wraps;
  EXPECT_EQ(index,expected);hint=index;BehaviorContext c;c.path={&global.path(),index};c.vehicle.localization_valid=true;
  c.vehicle.position={p.x,p.y};c.vehicle.yaw=p.yaw;c.free_space={3,3};
  const auto plan=planner.plan(Behavior::CRUISE,c);ASSERT_TRUE(plan.feasible)<<index;
  EXPECT_EQ(plan.path.size(),300u);
 }
 EXPECT_EQ(wraps,3);
}
TEST(ClosedLoop, LoadingDuplicateEndpointUsesPeriodicGeometry) {
 const std::string filename="/tmp/planning_closed_circle_regression.csv";const auto circle=circleRoute();
 {std::ofstream f(filename);f.precision(17);for(const auto &p:circle)f<<p.x<<","<<p.y<<",0,0\n";f<<circle.front().x<<","<<circle.front().y<<",0,0\n";}
 GlobalPath global;global.setClosed(true);std::string error;ASSERT_TRUE(global.load(filename,&error,.75))<<error;std::remove(filename.c_str());
 EXPECT_EQ(global.routeSize(),circle.size());
 for(auto index:{size_t(0),size_t(1),global.routeSize()-1}){
  EXPECT_NEAR(global.path()[index].curvature,1./8,.02);
  const auto &a=global.path()[index];const auto &b=global.path()[index+1];
  EXPECT_LT(std::hypot(a.x-b.x,a.y-b.y),.1);
 }
}
TEST(ClosedLoop, RejectsUnmappedClosingGap) {
 const std::string filename="/tmp/planning_open_gap_regression.csv";
 {std::ofstream f(filename);f<<"0,0,0,0\n1,0,0,0\n2,0,0,0\n";}
 GlobalPath global;global.setClosed(true);std::string error;EXPECT_FALSE(global.load(filename,&error));std::remove(filename.c_str());
 EXPECT_NE(error.find("endpoint gap"),std::string::npos);
}
}

namespace planning {
TEST(AutoEntry, UnknownAndManualModeNeverQualify) {
 AutoEntryGuard guard;auto now=std::chrono::steady_clock::now();
 EXPECT_FALSE(guard.update(true,true,true,.1,0,now));
 guard.observeMode(0,now);
 for(int i=0;i<100;++i)EXPECT_FALSE(guard.update(true,true,true,.1,0,now+std::chrono::milliseconds(i*50)));
 EXPECT_TRUE(guard.holding());
}
TEST(AutoEntry, RecordedFarManualToAutoReentryRemainsStopped) {
 AutoEntryGuard guard;auto now=std::chrono::steady_clock::now();guard.observeMode(0,now);
 guard.observeMode(1,now+std::chrono::seconds(5));
 for(int i=0;i<200;++i)EXPECT_FALSE(guard.update(true,true,true,2.816,22.13*M_PI/180,
   now+std::chrono::milliseconds(5000+i*50)));
 EXPECT_TRUE(guard.holding());
}
TEST(AutoEntry, QualifiesStoppedAlignedThenCapsAndRequalifiesAfterModeEdge) {
 AutoEntryGuard guard;auto now=std::chrono::steady_clock::now();guard.observeMode(1,now);
 for(int i=0;i<29;++i)EXPECT_FALSE(guard.update(true,true,true,.1,.05,now+std::chrono::milliseconds(i*50)));
 EXPECT_TRUE(guard.update(true,true,true,.1,.05,now+std::chrono::milliseconds(1450)));
 EXPECT_TRUE(guard.lowSpeed(now+std::chrono::milliseconds(1500)));
 EXPECT_DOUBLE_EQ(guard.speedCap(),.5/3.6);
 EXPECT_FALSE(guard.lowSpeed(now+std::chrono::milliseconds(2500)));
 // Normal avoidance must not be mistaken for a fresh mode entry.
 EXPECT_TRUE(guard.update(true,false,true,1.8,.6,now+std::chrono::milliseconds(2550)));
 guard.observeMode(0,now+std::chrono::milliseconds(2600));
 guard.observeMode(1,now+std::chrono::milliseconds(2610));
 EXPECT_FALSE(guard.update(true,true,true,.1,.05,now+std::chrono::milliseconds(2650)));
}
TEST(AutoEntry, StaleMovingMisalignedAndInfeasibleCannotQualify) {
 AutoEntryGuard guard;auto now=std::chrono::steady_clock::now();guard.observeMode(1,now);
 for(int i=0;i<80;++i) {
  EXPECT_FALSE(guard.update(i%4!=0,i%4!=1,i%4!=2,.1,i%4==3?1.:0.,now+std::chrono::milliseconds(i*50)));
 }
 guard.observeMode(-1,now+std::chrono::seconds(10));
 EXPECT_FALSE(guard.update(true,true,true,.1,0,now+std::chrono::seconds(20)));
}
TEST(RecoveryGuard, ManualModeCannotReleaseRecoveryEvenWithValidPath) {
 RecoveryGuard guard;BehaviorContext c;c.vehicle.localization_valid=true;
 auto now=std::chrono::steady_clock::now();
 for(int i=0;i<100;++i)EXPECT_EQ(guard.apply(c,Behavior::CRUISE,true,true,true,
  now+std::chrono::milliseconds(i*50),nullptr,false),Behavior::EMERGENCY_STOP);
 EXPECT_TRUE(guard.holding());
}
}
