#include "planning_pkg_2025/local/local_planner.hpp"

#include <cmath>
#include <utility>

namespace planning
{
LocalPlanner::LocalPlanner()
: LocalPlanner(PathGenerator(PathGeneratorConfig{}), CollisionChecker(CollisionConfig{}),
    CostEvaluator(CostConfig{})) {}

LocalPlanner::LocalPlanner(PathGenerator generator, CollisionChecker checker, CostEvaluator evaluator)
: generator_(std::move(generator)), checker_(std::move(checker)), evaluator_(std::move(evaluator)) {}

LocalPlan LocalPlanner::plan(Behavior behavior, const BehaviorContext & context) const
{
  LocalPlan result;
  if (!context.path.global_path || context.path.global_path->empty()) {return result;}
  const bool avoidance = behavior == Behavior::AVOID;
  const auto & reference_point = context.path.global_path->at(context.path.nearest_index);
  const double dx = context.vehicle.position.x - reference_point.x;
  const double dy = context.vehicle.position.y - reference_point.y;
  const double start_lateral_offset = -std::sin(reference_point.yaw) * dx +
    std::cos(reference_point.yaw) * dy;
  result.candidates = generator_.generate(
    *context.path.global_path, context.path.nearest_index, avoidance, start_lateral_offset);
  CandidatePath * best = nullptr;
  for (auto & candidate : result.candidates) {
    if (!checker_.validate(candidate, context)) {continue;}
    evaluator_.evaluate(candidate);
    if (!best || candidate.cost < best->cost) {best = &candidate;}
  }
  if (best) {
    result.path = best->path;
    result.selected_offset = best->lateral_offset;
    result.feasible = true;
  }
  return result;
}
}  // namespace planning
