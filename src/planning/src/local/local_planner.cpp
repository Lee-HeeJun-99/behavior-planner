#include "planning_pkg_2025/local/local_planner.hpp"

#include <cmath>
#include <utility>

namespace planning
{
LocalPlanner::LocalPlanner()
: LocalPlanner(PathGenerator(PathGeneratorConfig{}), CollisionChecker(CollisionConfig{}),
    CostEvaluator(CostConfig{})) {}

LocalPlanner::LocalPlanner(PathGenerator generator, CollisionChecker checker, CostEvaluator evaluator)
: generator_(std::move(generator)), generator_config_(generator_.config()), checker_(std::move(checker)), evaluator_(std::move(evaluator)) {}

LocalPlan LocalPlanner::plan(Behavior behavior, const BehaviorContext & context) const
{
  LocalPlan result;
  if (!context.path.global_path || context.path.global_path->empty()) {return result;}
  const bool avoidance = behavior == Behavior::AVOID;
  if (avoidance != committed_is_avoidance_) {committed_avoidance_path_.clear();}
  if (!committed_avoidance_path_.empty()) {
    std::size_t nearest = 0;
    double distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < committed_avoidance_path_.size(); ++i) {
      const auto & point = committed_avoidance_path_[i];
      const double d = std::hypot(point.x - context.vehicle.position.x, point.y - context.vehicle.position.y);
      if (d < distance) {distance = d; nearest = i;}
    }
    if (distance <= 0.4 && committed_avoidance_path_.size() - nearest >= 3) {
      CandidatePath retained;
      retained.lateral_offset = committed_offset_;
      retained.path.assign(committed_avoidance_path_.begin() + nearest, committed_avoidance_path_.end());
      if (checker_.validate(retained, context)) {
        evaluator_.evaluate(retained);
        result.path = retained.path;
        result.selected_offset = committed_offset_;
        result.feasible = true;
        result.candidates.push_back(std::move(retained));
        return result;
      }
    }
    committed_avoidance_path_.clear();
  }
  const auto & reference_point = context.path.global_path->at(context.path.nearest_index);
  const double dx = context.vehicle.position.x - reference_point.x;
  const double dy = context.vehicle.position.y - reference_point.y;
  const double start_lateral_offset = -std::sin(reference_point.yaw) * dx +
    std::cos(reference_point.yaw) * dy;
  const auto & reference = *context.path.global_path;
  double nearest_ahead = std::numeric_limits<double>::infinity();
  double return_start = 0.0;
  for (const auto & obstacle : context.obstacles) {
    const double ox = obstacle.center.x - reference_point.x;
    const double oy = obstacle.center.y - reference_point.y;
    const double along = std::cos(reference_point.yaw) * ox + std::sin(reference_point.yaw) * oy;
    const double lateral = -std::sin(reference_point.yaw) * ox + std::cos(reference_point.yaw) * oy;
    if (along >= 0.0 && std::abs(lateral) <= 0.8 + obstacle.radius) {
      nearest_ahead = std::min(nearest_ahead, along);
    }
    // Arc distance locates the return after the obstacle, including curved roads.
    std::size_t closest = context.path.nearest_index;
    double best_distance = std::numeric_limits<double>::infinity();
    const std::size_t end = std::min(reference.size(), closest + generator_config_.horizon_points + 200);
    for (std::size_t j = closest; j < end; ++j) {
      const double distance = std::hypot(reference[j].x - obstacle.center.x, reference[j].y - obstacle.center.y);
      if (distance < best_distance) {best_distance = distance; closest = j;}
    }
    double arc = 0.0;
    for (std::size_t j = context.path.nearest_index + 1; j <= closest; ++j) {
      arc += std::hypot(reference[j].x - reference[j - 1].x, reference[j].y - reference[j - 1].y);
    }
    return_start = std::max(return_start, arc + obstacle.radius + generator_config_.obstacle_pass_margin);
  }
  Path approach_reference;
  const Path * planning_reference = &reference;
  if (!avoidance && nearest_ahead > generator_config_.avoidance_start_distance &&
    std::isfinite(nearest_ahead)) {
    // Approach on the center path, ending before the obstacle's safety buffer.
    const double available = nearest_ahead - generator_config_.obstacle_pass_margin;
    std::size_t end = context.path.nearest_index + 1;
    double arc = 0.0;
    while (end < reference.size()) {
      arc += std::hypot(reference[end].x - reference[end - 1].x, reference[end].y - reference[end - 1].y);
      if (arc >= available) {break;}
      ++end;
    }
    approach_reference.assign(reference.begin(), reference.begin() + end);
    planning_reference = &approach_reference;
  }
  result.candidates = generator_.generate(
    *planning_reference, context.path.nearest_index, avoidance, start_lateral_offset,
    avoidance ? return_start : -1.0);
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
    if (avoidance || std::abs(start_lateral_offset) > 0.15) {
      committed_is_avoidance_ = avoidance;
      committed_avoidance_path_ = result.path;
      committed_offset_ = result.selected_offset;
    }
  }
  return result;
}
}  // namespace planning
