#include "planning_pkg_2025/behavior/behavior_planner.hpp"

#include <algorithm>
#include <cmath>

namespace planning
{
BehaviorPlanner::BehaviorPlanner(BehaviorConfig config) : config_(config) {}

bool BehaviorPlanner::pathBlocked(const BehaviorContext & context) const
{
  if (!context.path.global_path || context.path.global_path->empty()) {return false;}
  const auto & path = *context.path.global_path;
  const std::size_t end = std::min(path.size(), context.path.nearest_index + 350);
  for (const auto & obstacle : context.obstacles) {
    const auto & ref = path[context.path.nearest_index];
    const double dx = obstacle.center.x - ref.x;
    const double dy = obstacle.center.y - ref.y;
    const double along = std::cos(ref.yaw) * dx + std::sin(ref.yaw) * dy;
    const double lateral = -std::sin(ref.yaw) * dx + std::cos(ref.yaw) * dy;
    if (current_ == Behavior::AVOID && along >= -config_.obstacle_pass_margin - obstacle.radius &&
      along < 0.0 && std::abs(lateral) <= config_.obstacle_corridor_half_width + obstacle.radius) {
      return true;
    }
    if (along < 0.0) {continue;}
    for (std::size_t i = context.path.nearest_index; i < end; ++i) {
      const double along = std::hypot(path[i].x - context.vehicle.position.x, path[i].y - context.vehicle.position.y);
      if (along > config_.obstacle_lookahead) {break;}
      if (std::hypot(path[i].x - obstacle.center.x, path[i].y - obstacle.center.y) <=
        config_.obstacle_corridor_half_width + obstacle.radius) {return true;}
    }
  }
  return false;
}

void BehaviorPlanner::transition(Behavior next, std::chrono::steady_clock::time_point now)
{
  if (next != current_) {current_ = next; last_transition_ = now;}
}

Behavior BehaviorPlanner::update(const BehaviorContext & context, std::chrono::steady_clock::time_point now)
{
  if (!context.vehicle.localization_valid || context.emergency_stop) {
    transition(Behavior::EMERGENCY_STOP, now); return current_;
  }
  const bool blocked = pathBlocked(context);
  detection_count_ = blocked ? detection_count_ + 1 : 0;
  clear_count_ = blocked ? 0 : clear_count_ + 1;
  const bool duration_met = last_transition_.time_since_epoch().count() == 0 ||
    now - last_transition_ >= config_.minimum_behavior_duration;
  if (current_ == Behavior::AVOID) {
    if (clear_count_ >= config_.clear_confirmation_count && duration_met) {transition(Behavior::CRUISE, now);}
  } else if (detection_count_ >= config_.detection_confirmation_count && duration_met) {
    transition(Behavior::AVOID, now);
  } else if (current_ == Behavior::STOP || current_ == Behavior::WAIT || current_ == Behavior::EMERGENCY_STOP) {
    transition(Behavior::CRUISE, now);
  }
  return current_;
}
}  // namespace planning
