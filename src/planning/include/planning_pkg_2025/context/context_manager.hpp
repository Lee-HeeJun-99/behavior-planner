#pragma once

#include <vector>

#include "planning_pkg_2025/common/planning_types.hpp"

namespace planning
{
struct ContextConfig
{
  double obstacle_radius{0.35};
  double obstacle_relevance_distance{20.0};
  double road_left_bound{1.5};
  double road_right_bound{1.5};
  int default_speed_limit_kph{20};
  double default_target_velocity{-1.0};
  int speed_sign_confirmation_count{3};
  double speed_limit_20_target{2.5};
  double speed_limit_50_target{4.5};
  double avoid_max_velocity{2.5};
};

class ContextManager
{
public:
  explicit ContextManager(ContextConfig config = {});
  void updateVehicle(const Point2d & position, double yaw);
  void updateObstacles(const std::vector<Point2d> & points);
  void updateSpeedLimitDetection(int detected_limit_kph);
  void setEmergencyStop(bool active) {emergency_stop_ = active;}
  BehaviorContext build(const Path & global_path, std::size_t nearest_index) const;

private:
  ContextConfig config_;
  VehicleContext vehicle_;
  std::vector<Point2d> raw_obstacles_;
  SpeedContext speed_;
  int speed_candidate_kph_{0};
  int speed_candidate_count_{0};
  bool emergency_stop_{false};

  bool isSupportedSpeedLimit(int limit_kph) const;
  double targetVelocityForLimit(int limit_kph) const;
};
}  // namespace planning
