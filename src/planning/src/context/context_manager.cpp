#include "planning_pkg_2025/context/context_manager.hpp"

#include <cmath>
#include <utility>

namespace planning
{
namespace {double distance(const Point2d & a, const Point2d & b) {return std::hypot(a.x - b.x, a.y - b.y);}}

ContextManager::ContextManager(ContextConfig config) : config_(config) {}

void ContextManager::updateVehicle(const Point2d & position, double yaw)
{
  vehicle_.position = position;
  vehicle_.yaw = yaw;
  vehicle_.localization_valid = std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(yaw);
}

void ContextManager::updateObstacles(const std::vector<Point2d> & points) {raw_obstacles_ = points;}
void ContextManager::setStopPoints(std::vector<Point2d> stop_points) {stop_points_ = std::move(stop_points);}

BehaviorContext ContextManager::build(const Path & global_path, std::size_t nearest_index) const
{
  BehaviorContext context;
  context.vehicle = vehicle_;
  context.path = {&global_path, nearest_index};
  context.free_space = {config_.road_left_bound, config_.road_right_bound};
  context.emergency_stop = emergency_stop_;
  for (const auto & point : raw_obstacles_) {
    if (distance(vehicle_.position, point) <= config_.obstacle_relevance_distance) {
      context.obstacles.push_back({point, config_.obstacle_radius});
    }
  }
  for (const auto & stop : stop_points_) {
    const double d = distance(vehicle_.position, stop);
    if (d < context.stop.distance) {context.stop.distance = d;}
  }
  context.stop.stop_required = context.stop.distance <= config_.stop_trigger_distance;
  context.stop.stop_completed = context.stop.distance > config_.stop_release_distance;
  return context;
}
}  // namespace planning
