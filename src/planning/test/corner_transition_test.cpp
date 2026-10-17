#include <algorithm>
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include "planning_pkg_2025/local/local_planner.hpp"

namespace planning
{
namespace
{
PathGeneratorConfig cornerConfig()
{
  PathGeneratorConfig config;
  config.transition_points = 120;
  config.lateral_offsets = {-1.8, -1.5, -1.2, -0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8};
  return config;
}
void checkRecordedCorner(const std::string & filename)
{
  std::ifstream file(std::string(PLANNING_TEST_DATA_DIR) + "/" + filename);
  ASSERT_TRUE(file.good());
  BehaviorContext context;
  context.vehicle.localization_valid = true;
  std::size_t obstacle_count = 0;
  file >> context.vehicle.position.x >> context.vehicle.position.y >> context.vehicle.yaw >> obstacle_count;
  for (std::size_t i = 0; i < obstacle_count; ++i) {
    ObstacleContext obstacle;
    file >> obstacle.center.x >> obstacle.center.y;
    obstacle.radius = 0.35;
    context.obstacles.push_back(obstacle);
  }
  Path reference;
  PathPoint point;
  while (file >> point.x >> point.y >> point.yaw >> point.curvature) {reference.push_back(point);}
  ASSERT_EQ(reference.size(), 300u);
  context.path = {&reference, 0};
  context.free_space = {3.0, 3.0};
  const auto config = cornerConfig();
  CollisionConfig collision;
  collision.vehicle_half_width = 0.6;
  collision.vehicle_front = 1.6;
  collision.vehicle_rear = 0.7;
  collision.safety_margin = 0.25;
  collision.maximum_curvature = 0.45;
  LocalPlanner planner{PathGenerator(config), CollisionChecker(collision), CostEvaluator{}};
  const auto plan = planner.plan(Behavior::AVOID, context);
  ASSERT_TRUE(plan.feasible);
  ASSERT_EQ(plan.path.size(), 300u);  // The bend cannot be hidden by shortening the horizon.
  ASSERT_GE(plan.candidates.size(), config.lateral_offsets.size());
  for (std::size_t i = 0; i < config.lateral_offsets.size(); ++i) {
    EXPECT_FALSE(plan.candidates[i].valid);  // Regression: all original profiles fail.
  }
  for (const auto & p : plan.path) {EXPECT_LE(std::abs(p.curvature), collision.maximum_curvature);}
  CandidatePath independent;
  independent.path = plan.path;
  independent.lateral_offset = plan.selected_offset;
  EXPECT_TRUE(CollisionChecker(collision).validate(independent, context));
  // A new obstacle on the selected path must invalidate the cached path too.
  const auto & blocking_point = plan.path[100];
  context.obstacles.push_back({{blocking_point.x, blocking_point.y}, 20.0});
  EXPECT_FALSE(planner.plan(Behavior::AVOID, context).feasible);
}
}  // namespace

TEST(CornerTransition, Recorded1017SixAvoidsObstacleWithoutExceedingCurvature)
{
  checkRecordedCorner("right_turn_obstacle_1017_6.txt");
}
TEST(CornerTransition, Recorded1017FourAvoidsObstacleWithoutExceedingCurvature)
{
  checkRecordedCorner("right_turn_obstacle_1017_4.txt");
}
TEST(CornerTransition, IntermediateOffsetMustRemainInsideRoad)
{
  CandidatePath candidate;
  candidate.path = {{0, 0, 0, 0}, {1, 0, 0, 0}, {2, 0, 0, 0}};
  candidate.lateral_offset = 0.0;
  candidate.lateral_profile = {0.0, 2.2, 0.0};
  BehaviorContext context;
  context.free_space = {3.0, 3.0};
  CollisionConfig config;
  config.vehicle_half_width = 0.6;
  config.safety_margin = 0.25;
  EXPECT_FALSE(CollisionChecker(config).validate(candidate, context));
  EXPECT_EQ(candidate.invalid_reason, "static_boundary");
}
TEST(CornerTransition, CostIncludesCornerOffsetInsteadOfOnlyPassingOffset)
{
  CandidatePath centered, shifted;
  centered.valid = shifted.valid = true;
  centered.path = shifted.path = {{0, 0, 0, 0}, {1, 0, 0, 0}, {2, 0, 0, 0}};
  centered.lateral_profile = {0.0, 0.0, 0.0};
  shifted.lateral_profile = {0.0, 0.6, 1.8};
  EXPECT_GT(CostEvaluator{}.evaluate(shifted), CostEvaluator{}.evaluate(centered));
}
}  // namespace planning
