#pragma once

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

struct StopContext
{
  bool stop_required{false};
  bool stop_completed{false};
  double distance{std::numeric_limits<double>::infinity()};
};

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
  StopContext stop;
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

struct CandidatePath
{
  Path path;
  double lateral_offset{0.0};
  double minimum_clearance{std::numeric_limits<double>::infinity()};
  double cost{std::numeric_limits<double>::infinity()};
  bool valid{false};
  std::string invalid_reason;
};

}  // namespace planning
