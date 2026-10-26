#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include "planning_pkg_2025/local/local_planner.hpp"

namespace planning {
namespace {
PathGeneratorConfig generation() {
  PathGeneratorConfig g;
  g.transition_points=120;
  g.avoidance_start_distance=5;
  g.lateral_offsets={-1.8,-1.5,-1.2,-.9,-.6,-.3,0.,.3,.6,.9,1.2,1.5,1.8};
  return g;
}
CollisionConfig vehicle() {
  CollisionConfig c;
  c.vehicle_front=1.05;c.vehicle_rear=.95;c.vehicle_half_width=.45;
  c.maximum_curvature=.5;
  return c;
}
LocalPlanner planner() {
  return LocalPlanner{PathGenerator(generation()),CollisionChecker(vehicle()),CostEvaluator{}};
}
}

TEST(AdaptiveTransition, OffsetReturnScenarioUsesFullValidHorizon) {
  std::ifstream file(std::string(PLANNING_TEST_DATA_DIR)+"/offset_return_1026_7_scenario.txt");
  ASSERT_TRUE(file.good());
  BehaviorContext c;std::size_t count;
  file>>c.vehicle.position.x>>c.vehicle.position.y>>c.vehicle.yaw>>count;
  ASSERT_EQ(count,0u);
  Path ref;PathPoint point;
  while(file>>point.x>>point.y>>point.yaw>>point.curvature)ref.push_back(point);
  ASSERT_EQ(ref.size(),300u);
  c.vehicle.localization_valid=true;c.free_space={3,3};c.path={&ref,0};
  auto local=planner();const auto result=local.plan(Behavior::CRUISE,c);
  ASSERT_TRUE(result.feasible);
  ASSERT_EQ(result.path.size(),300u);
  ASSERT_GT(result.candidates.size(),1u);
  EXPECT_EQ(result.candidates.front().invalid_reason,"curvature");
  CandidatePath checked;checked.path=result.path;checked.lateral_offset=result.selected_offset;
  EXPECT_TRUE(CollisionChecker(vehicle()).validate(checked,c));
  EXPECT_NEAR(result.path.front().x,c.vehicle.position.x,1e-6);
  EXPECT_NEAR(result.path.front().y,c.vehicle.position.y,1e-6);
  for(auto &p:result.path)EXPECT_LE(std::abs(p.curvature),.5);
  // Cached recovery must be invalidated when a new obstacle blocks the route.
  c.obstacles.push_back({{result.path[80].x,result.path[80].y},20.});
  EXPECT_FALSE(local.plan(Behavior::CRUISE,c).feasible);
}

TEST(AdaptiveTransition, CloseObstacleKeepsCollisionAndClearanceConstraints) {
  Path ref;for(int i=0;i<400;++i)ref.push_back({i*.05,0,0,0});
  BehaviorContext c;c.vehicle.localization_valid=true;c.path={&ref,0};c.free_space={3,3};
  for(double distance:{2.,3.}) {
    auto local=planner();c.obstacles={{{distance,0},.35}};
    EXPECT_FALSE(local.plan(Behavior::AVOID,c).feasible);
  }
  auto local=planner();c.obstacles={{{4.,0},.35}};
  auto result=local.plan(Behavior::AVOID,c);
  ASSERT_TRUE(result.feasible);ASSERT_EQ(result.path.size(),300u);
  CandidatePath checked;checked.path=result.path;checked.lateral_offset=result.selected_offset;
  EXPECT_TRUE(CollisionChecker(vehicle()).validate(checked,c));
}
TEST(AdaptiveTransition, LastObstaclePoseAssumptionDoesNotBypassClearance) {
  // Centers are reprocessed from bag LiDAR. Pose is a route-index assumption,
  // since 1026-7 does not contain the vehicle position topic.
  std::ifstream file(std::string(PLANNING_TEST_DATA_DIR)+"/last_obstacle_1026_7_pose_assumed.txt");
  ASSERT_TRUE(file.good());
  BehaviorContext c;std::size_t count;
  file>>c.vehicle.position.x>>c.vehicle.position.y>>c.vehicle.yaw>>count;
  ASSERT_EQ(count,2u);
  for(std::size_t i=0;i<count;++i){ObstacleContext o;file>>o.center.x>>o.center.y;o.radius=.35;c.obstacles.push_back(o);}
  Path ref;PathPoint point;
  while(file>>point.x>>point.y>>point.yaw>>point.curvature)ref.push_back(point);
  ASSERT_EQ(ref.size(),300u);
  c.path={&ref,0};c.free_space={3,3};c.vehicle.localization_valid=true;
  auto local=planner();auto result=local.plan(Behavior::AVOID,c);
  EXPECT_FALSE(result.feasible);
  EXPECT_TRUE(result.path.empty());
}
}  // namespace planning
