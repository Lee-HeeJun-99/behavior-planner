#include <gtest/gtest.h>
#include "planning_pkg_2025/local/local_planner.hpp"
namespace planning {
namespace {
Path line() {Path p;for(int i=0;i<400;++i)p.push_back({i*.05,0,0,0});return p;}
LocalPlanner passingPlanner(double preferred) {
 PathGeneratorConfig g;g.transition_points=120;g.preferred_avoidance_offset=preferred;
 g.lateral_offsets={-1.8,-1.5,-1.2,-.9,-.6,-.3,0.,.3,.6,.9,1.2,1.5,1.8};
 CollisionConfig c;c.vehicle_front=1.05;c.vehicle_rear=.95;c.vehicle_half_width=.45;c.safety_margin=0;c.minimum_clearance=0;c.maximum_curvature=.5;
 return LocalPlanner{PathGenerator(g),CollisionChecker(c),CostEvaluator{}};
}
}
TEST(PreferredAvoidanceOffset, PrefersWideValidatedPassAroundRightObstacle) {
 auto ref=line();BehaviorContext c;c.vehicle.localization_valid=true;c.path={&ref,0};c.free_space={3,3};c.obstacles={{{5,-.6},.5}};
 auto local=passingPlanner(1.2);auto plan=local.plan(Behavior::AVOID,c);
 ASSERT_TRUE(plan.feasible);EXPECT_GE(plan.selected_offset,1.2);ASSERT_EQ(plan.path.size(),300u);
 for(auto &p:plan.path)EXPECT_LE(std::abs(p.curvature),.5);
 c.obstacles.push_back({c.vehicle.position,1.});EXPECT_FALSE(local.plan(Behavior::AVOID,c).feasible);
}
TEST(PreferredAvoidanceOffset, CanUseNarrowerValidCandidateWhenRoadHasNoWideCandidate) {
 auto ref=line();BehaviorContext c;c.vehicle.localization_valid=true;c.path={&ref,0};c.free_space={1.4,1.4};c.obstacles={{{5,-.6},.5}};
 auto local=passingPlanner(1.2);auto plan=local.plan(Behavior::AVOID,c);
 ASSERT_TRUE(plan.feasible);EXPECT_LT(std::abs(plan.selected_offset),1.2);
}
TEST(PreferredAvoidanceOffset, DoesNotShiftCruisePath) {
 auto ref=line();BehaviorContext c;c.vehicle.localization_valid=true;c.path={&ref,0};c.free_space={3,3};
 auto local=passingPlanner(1.2);auto plan=local.plan(Behavior::CRUISE,c);
 ASSERT_TRUE(plan.feasible);EXPECT_DOUBLE_EQ(plan.selected_offset,0.);
}
}  // namespace planning
