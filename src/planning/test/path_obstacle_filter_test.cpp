#include <limits>
#include <gtest/gtest.h>
#include "planning_pkg_2025/context/context_manager.hpp"
#include "planning_pkg_2025/behavior/recovery_guard.hpp"

namespace planning {
namespace {
ContextConfig routeFilter() {
  ContextConfig c;c.filter_obstacles_by_path=true;c.obstacle_radius=.5;
  c.road_left_bound=c.road_right_bound=3;
  c.obstacle_path_lookahead=6;c.obstacle_path_lookbehind=1.5;
  return c;
}
}
TEST(PathObstacleFilter, UsesFixedSizeWithinThreeMeterRoadAndRejectsDistantCenters) {
  Path path={{0,0,0,0},{10,0,0,0}};
  ContextManager manager(routeFilter());manager.updateVehicle({0,0},0);
  manager.updateObstacles({{3,2.9},{3,3.1},{6.6,0},{-.5,0},{-2.1,0},
    {3,std::numeric_limits<double>::quiet_NaN()}});
  auto context=manager.build(path,0);
  ASSERT_EQ(context.obstacles.size(),2u);
  EXPECT_DOUBLE_EQ(context.obstacles[0].radius,.5);
  EXPECT_DOUBLE_EQ(context.obstacles[0].center.y,2.9);
  EXPECT_DOUBLE_EQ(context.obstacles[1].center.x,-.5);
}
TEST(PathObstacleFilter, MatchesSegmentsAndFollowsActiveCornerInsteadOfOtherBranches) {
  Path path={{0,0,0,0},{4,0,1.57,0},{4,4,3.14,0},{0,4,0,0}};
  ContextManager manager(routeFilter());manager.updateVehicle({0,0},.7853981633974483);
  manager.updateObstacles({{2,0},{4,1},{7.1,1},{2,4}});
  auto context=manager.build(path,0);
  ASSERT_EQ(context.obstacles.size(),2u);
  EXPECT_DOUBLE_EQ(context.obstacles[1].center.x,4);
  EXPECT_DOUBLE_EQ(context.obstacles[1].center.y,1);
}
TEST(PathObstacleFilter, RouteSideDetectionsCannotHoldRecovery) {
  Path path={{0,0,0,0},{10,0,0,0}};
  ContextManager manager(routeFilter());manager.updateVehicle({0,0},0);
  manager.updateObstacles({{1,3.1},{3,-3.1}});
  auto context=manager.build(path,0);ASSERT_TRUE(context.obstacles.empty());
  RecoveryGuard guard;
  EXPECT_EQ(guard.apply(context,Behavior::CRUISE,true,true,true,
    std::chrono::steady_clock::now()),Behavior::CRUISE);
  EXPECT_FALSE(guard.holding());
}
TEST(PathObstacleFilter, DisabledFilterPreservesAllRelevantInputs) {
  auto config=routeFilter();config.filter_obstacles_by_path=false;
  ContextManager manager(config);manager.updateVehicle({0,0},0);
  manager.updateObstacles({{3,2},{3,-2}});
  EXPECT_EQ(manager.build({{0,0,0,0},{10,0,0,0}},0).obstacles.size(),2u);
}
}  // namespace planning
