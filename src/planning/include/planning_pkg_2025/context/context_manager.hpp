#pragma once

#include <vector>

#include "planning_pkg_2025/common/planning_types.hpp"

namespace planning
{
struct ContextConfig
{
  double obstacle_radius{0.35};
  double obstacle_relevance_distance{20.0};
  double stop_trigger_distance{1.0};
  double stop_release_distance{2.0};
  double road_left_bound{1.5};
  double road_right_bound{1.5};
};

class ContextManager
{
public:
  explicit ContextManager(ContextConfig config = {});
  void updateVehicle(const Point2d & position, double yaw);
  void updateObstacles(const std::vector<Point2d> & points);
  void setStopPoints(std::vector<Point2d> stop_points);
  void setEmergencyStop(bool active) {emergency_stop_ = active;}
  BehaviorContext build(const Path & global_path, std::size_t nearest_index) const;

private:
  ContextConfig config_;
  VehicleContext vehicle_;
  std::vector<Point2d> raw_obstacles_;
  std::vector<Point2d> stop_points_;
  bool emergency_stop_{false};
};
}  // namespace planning
