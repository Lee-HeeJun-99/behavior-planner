#include <gtest/gtest.h>
#include "planning_pkg_2025/local/collision_checker.hpp"
namespace planning {
namespace {
CollisionConfig vehicleClearance(double clearance) {
  CollisionConfig c;c.vehicle_front=1.05;c.vehicle_rear=.95;c.vehicle_half_width=.45;
  c.safety_margin=.25;c.maximum_curvature=.5;c.minimum_clearance=clearance;return c;
}
CandidatePath straightCandidate() {
  CandidatePath candidate;for(int i=0;i<100;++i)candidate.path.push_back({i*.05,0,0,0});return candidate;
}
}
TEST(ClearanceParameter, BagOneSideDistancePassesFiveCentimetersExtraClearance) {
  BehaviorContext context;context.free_space={3,3};context.obstacles={{{1,-1.30},.5}};
  auto candidate=straightCandidate();
  EXPECT_FALSE(CollisionChecker(vehicleClearance(.15)).validate(candidate,context));
  EXPECT_EQ(candidate.invalid_reason,"minimum_clearance");
  EXPECT_TRUE(CollisionChecker(vehicleClearance(.05)).validate(candidate,context));
  EXPECT_NEAR(candidate.minimum_clearance,.10,1e-6);
}
TEST(ClearanceParameter, InflatedCollisionFootprintRemainsBlocked) {
  BehaviorContext context;context.free_space={3,3};context.obstacles={{{1,-1.19},.5}};
  auto candidate=straightCandidate();
  EXPECT_FALSE(CollisionChecker(vehicleClearance(.05)).validate(candidate,context));
  EXPECT_EQ(candidate.invalid_reason,"obstacle_collision");
}
TEST(ClearanceParameter, LessThanFiveCentimetersStillFails) {
  BehaviorContext context;context.free_space={3,3};context.obstacles={{{1,-1.23},.5}};
  auto candidate=straightCandidate();
  EXPECT_FALSE(CollisionChecker(vehicleClearance(.05)).validate(candidate,context));
  EXPECT_EQ(candidate.invalid_reason,"minimum_clearance");
}
}  // namespace planning
