#include "planning_pkg_2025/local/collision_checker.hpp"

#include <algorithm>
#include <cmath>

namespace planning
{
CollisionChecker::CollisionChecker(CollisionConfig config) : config_(config) {}

bool CollisionChecker::validate(CandidatePath & candidate, const BehaviorContext & context) const
{
  candidate.valid = false;
  if (candidate.path.size() < 3) {candidate.invalid_reason = "path_too_short"; return false;}
  if (candidate.lateral_offset > context.free_space.left_bound - config_.vehicle_half_width - config_.safety_margin ||
    candidate.lateral_offset < -context.free_space.right_bound + config_.vehicle_half_width + config_.safety_margin) {
    candidate.invalid_reason = "static_boundary"; return false;
  }
  candidate.minimum_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < candidate.path.size(); ++i) {
    const auto & point = candidate.path[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.curvature)) {
      candidate.invalid_reason = "non_finite"; return false;
    }
    if (std::abs(point.curvature) > config_.maximum_curvature) {
      candidate.invalid_reason = "curvature"; return false;
    }
    for (const auto & obstacle : context.obstacles) {
      const double dx = obstacle.center.x - point.x;
      const double dy = obstacle.center.y - point.y;
      const double longitudinal = std::cos(point.yaw) * dx + std::sin(point.yaw) * dy;
      const double lateral = -std::sin(point.yaw) * dx + std::cos(point.yaw) * dy;
      const double length_limit = (longitudinal >= 0.0 ? config_.vehicle_front : config_.vehicle_rear) +
        config_.safety_margin + obstacle.radius;
      const double width_limit = config_.vehicle_half_width + config_.safety_margin + obstacle.radius;
      const double outside_longitudinal = std::max(std::abs(longitudinal) - length_limit, 0.0);
      const double outside_lateral = std::max(std::abs(lateral) - width_limit, 0.0);
      const double clearance = std::hypot(outside_longitudinal, outside_lateral);
      candidate.minimum_clearance = std::min(candidate.minimum_clearance, clearance);
      if (std::abs(longitudinal) <= length_limit && std::abs(lateral) <= width_limit) {
        candidate.invalid_reason = "obstacle_collision"; return false;
      }
    }
  }
  if (candidate.minimum_clearance < config_.minimum_clearance) {
    candidate.invalid_reason = "minimum_clearance"; return false;
  }
  candidate.valid = true;
  candidate.invalid_reason.clear();
  return true;
}
}  // namespace planning
