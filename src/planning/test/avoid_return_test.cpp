#include <gtest/gtest.h>
#include "planning_pkg_2025/behavior/behavior_planner.hpp"
namespace planning {
TEST(AvoidReturn, RemainsAvoidingUntilPositionAndHeadingRejoinRoute) {
  Path path={{0,0,0,0},{1,0,0,0},{2,0,0,0},{3,0,0,0}};
  BehaviorContext c;c.vehicle.localization_valid=true;c.path={&path,0};
  c.obstacles={{{2,0},.5}};
  BehaviorConfig cfg;cfg.detection_confirmation_count=1;cfg.clear_confirmation_count=2;
  cfg.minimum_behavior_duration=std::chrono::milliseconds(0);
  BehaviorPlanner planner(cfg);auto now=std::chrono::steady_clock::now();
  ASSERT_EQ(planner.update(c,now),Behavior::AVOID);
  c.obstacles.clear();c.vehicle.position={0,.6};
  for(int i=0;i<10;++i)EXPECT_EQ(planner.update(c,now),Behavior::AVOID);
  c.vehicle.position={0,.1};c.vehicle.yaw=.3;
  EXPECT_EQ(planner.update(c,now),Behavior::AVOID);
  c.vehicle.yaw=.05;
  EXPECT_EQ(planner.update(c,now),Behavior::CRUISE);
}
}  // namespace planning

#include <fstream>
#include "planning_pkg_2025/local/local_planner.hpp"
namespace planning {
TEST(AvoidReturn, BagThreeSideObstaclePassesWithFifteenCentimeterSafetyMargin) {
  // Pose is interpolated from bag TF; obstacle coordinates are recorded UTM.
  std::ifstream file(std::string(PLANNING_TEST_DATA_DIR)+"/bag3_side_obstacle_tf_pose.txt");
  ASSERT_TRUE(file.good());BehaviorContext c;std::size_t count;
  file>>c.vehicle.position.x>>c.vehicle.position.y>>c.vehicle.yaw>>count;
  for(std::size_t i=0;i<count;++i){ObstacleContext o;file>>o.center.x>>o.center.y;o.radius=.5;c.obstacles.push_back(o);}
  ASSERT_GT(c.obstacles.size(),0u);
  Path ref;PathPoint point;while(file>>point.x>>point.y>>point.yaw>>point.curvature)ref.push_back(point);
  ASSERT_EQ(ref.size(),300u);c.path={&ref,0};c.vehicle.localization_valid=true;c.free_space={3,3};
  PathGeneratorConfig g;g.transition_points=120;g.avoidance_start_distance=5;
  g.lateral_offsets={-1.8,-1.5,-1.2,-.9,-.6,-.3,0.,.3,.6,.9,1.2,1.5,1.8};
  CollisionConfig cfg;cfg.vehicle_front=1.05;cfg.vehicle_rear=.95;cfg.vehicle_half_width=.45;cfg.minimum_clearance=.05;cfg.maximum_curvature=.5;
  LocalPlanner previous{PathGenerator(g),CollisionChecker(cfg),CostEvaluator{}};
  EXPECT_FALSE(previous.plan(Behavior::AVOID,c).feasible);
  cfg.safety_margin=.15;LocalPlanner updated{PathGenerator(g),CollisionChecker(cfg),CostEvaluator{}};
  auto plan=updated.plan(Behavior::AVOID,c);ASSERT_TRUE(plan.feasible);ASSERT_EQ(plan.path.size(),300u);
  CandidatePath check;check.path=plan.path;check.lateral_offset=plan.selected_offset;
  EXPECT_TRUE(CollisionChecker(cfg).validate(check,c));
  c.obstacles.push_back({c.vehicle.position,.5});
  EXPECT_FALSE(updated.plan(Behavior::AVOID,c).feasible);
}
}  // namespace planning
