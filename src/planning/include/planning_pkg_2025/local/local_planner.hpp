#pragma once

#include "planning_pkg_2025/local/collision_checker.hpp"
#include "planning_pkg_2025/local/cost_evaluator.hpp"
#include "planning_pkg_2025/local/path_generator.hpp"

namespace planning
{
struct LocalPlan
{
  Path path;
  std::vector<CandidatePath> candidates;
  bool feasible{false};
  double selected_offset{0.0};
};

class LocalPlanner
{
public:
  LocalPlanner();
  LocalPlanner(PathGenerator generator, CollisionChecker checker, CostEvaluator evaluator);
  LocalPlan plan(Behavior behavior, const BehaviorContext & context) const;

private:
  PathGenerator generator_;
  CollisionChecker checker_;
  CostEvaluator evaluator_;
};
}  // namespace planning
