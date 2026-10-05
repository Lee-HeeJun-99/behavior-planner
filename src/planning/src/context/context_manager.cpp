#include "planning_pkg_2025/context/context_manager.hpp"

#include <algorithm>
#include <cmath>

namespace planning
{
namespace {double distance(const Point2d & a, const Point2d & b) {return std::hypot(a.x - b.x, a.y - b.y);}}

ContextManager::ContextManager(ContextConfig config) : config_(config)
{
  speed_.active_limit_kph = isSupportedSpeedLimit(config_.default_speed_limit_kph) ?
    config_.default_speed_limit_kph : 20;
  speed_.target_velocity = targetVelocityForLimit(speed_.active_limit_kph);
  speed_.avoid_max_velocity = std::max(0.0, config_.avoid_max_velocity);
}

void ContextManager::updateVehicle(const Point2d & position, double yaw)
{
  vehicle_.position = position;
  vehicle_.yaw = yaw;
  vehicle_.localization_valid = std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(yaw);
}

void ContextManager::updateObstacles(const std::vector<Point2d> & points) {raw_obstacles_ = points;}

bool ContextManager::isSupportedSpeedLimit(int limit_kph) const
{
  return limit_kph == 20 || limit_kph == 50;
}

double ContextManager::targetVelocityForLimit(int limit_kph) const
{
  if (limit_kph == 50) {return config_.speed_limit_50_target;}
  return config_.speed_limit_20_target;
}

void ContextManager::updateSpeedLimitDetection(int detected_limit_kph)
{
  speed_.detected_limit_kph = detected_limit_kph;
  speed_.detection_valid = isSupportedSpeedLimit(detected_limit_kph);
  if (!speed_.detection_valid) {
    speed_candidate_kph_ = 0;
    speed_candidate_count_ = 0;
    return;
  }
  if (speed_candidate_kph_ == detected_limit_kph) {
    ++speed_candidate_count_;
  } else {
    speed_candidate_kph_ = detected_limit_kph;
    speed_candidate_count_ = 1;
  }
  if (speed_candidate_count_ >= std::max(1, config_.speed_sign_confirmation_count)) {
    speed_.active_limit_kph = detected_limit_kph;
    speed_.target_velocity = targetVelocityForLimit(detected_limit_kph);
  }
}

BehaviorContext ContextManager::build(const Path & global_path, std::size_t nearest_index) const
{
  BehaviorContext context;
  context.vehicle = vehicle_;
  context.path = {&global_path, nearest_index};
  context.speed = speed_;
  context.free_space = {config_.road_left_bound, config_.road_right_bound};
  context.emergency_stop = emergency_stop_;
  for (const auto & point : raw_obstacles_) {
    if (distance(vehicle_.position, point) <= config_.obstacle_relevance_distance) {
      context.obstacles.push_back({point, config_.obstacle_radius});
    }
  }
  return context;
}
}  // namespace planning
