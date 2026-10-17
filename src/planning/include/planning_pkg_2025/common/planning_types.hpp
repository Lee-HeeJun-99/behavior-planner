#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace planning
{

struct Point2d
{
  double x{0.0};
  double y{0.0};
};

struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double curvature{0.0};
};

using Path = std::vector<PathPoint>;

struct VehicleContext
{
  Point2d position;
  double yaw{0.0};
  bool localization_valid{false};
};

struct PathContext
{
  const Path * global_path{nullptr};
  std::size_t nearest_index{0};
};

struct SpeedContext
{
  int detected_limit_kph{0};
  int active_limit_kph{20};
  double target_velocity{2.5};
  double avoid_max_velocity{2.5};
  bool detection_valid{false};
};

// Generic static detection, independent of semantic class. All relevant detections
// are available to local collision checks; behavior decides global-path relevance.
// TODO: carry measured bbox/radius from perception (current wire interface is XY only).
struct ObstacleContext
{
  Point2d center;
  double radius{0.0};
};

struct FreeSpaceContext
{
  double left_bound{1.5};
  double right_bound{1.5};
};

struct BehaviorContext
{
  VehicleContext vehicle;
  PathContext path;
  SpeedContext speed;
  std::vector<ObstacleContext> obstacles;
  FreeSpaceContext free_space;
  bool emergency_stop{false};
};

enum class Behavior { CRUISE, STOP, AVOID, WAIT, EMERGENCY_STOP };

inline const char * toString(Behavior behavior)
{
  switch (behavior) {
    case Behavior::CRUISE: return "CRUISE";
    case Behavior::STOP: return "STOP";
    case Behavior::AVOID: return "AVOID";
    case Behavior::WAIT: return "WAIT";
    case Behavior::EMERGENCY_STOP: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

inline double calculateTargetVelocity(Behavior behavior, const BehaviorContext & context)
{
  if (behavior == Behavior::STOP || behavior == Behavior::WAIT ||
    behavior == Behavior::EMERGENCY_STOP)
  {
    return 0.0;
  }
  if (behavior == Behavior::AVOID) {
    return std::min(context.speed.target_velocity, context.speed.avoid_max_velocity);
  }
  return context.speed.target_velocity;
}

struct CandidatePath
{
  Path path;
  double lateral_offset{0.0};
  std::vector<double> lateral_profile;
  double minimum_clearance{std::numeric_limits<double>::infinity()};
  double cost{std::numeric_limits<double>::infinity()};
  bool valid{false};
  std::string invalid_reason;
};

}  // namespace planning
