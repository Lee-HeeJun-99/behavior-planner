#pragma once

#include <cstddef>
#include <string>
#include <limits>

#include "planning_pkg_2025/common/planning_types.hpp"

namespace planning
{
class GlobalPath
{
public:
  bool load(const std::string & file_path, std::string * error = nullptr);
  bool load(const std::string & file_path, std::string * error, double smoothing_distance);
  // Configure before loading/setting the route.
  void setClosed(bool closed) {closed_ = closed;}
  bool closed() const {return closed_;}
  std::size_t routeSize() const {return lap_size_;}
  void setPath(Path path);
  const Path & path() const { return path_; }
  bool empty() const { return path_.empty(); }
  std::size_t nearestIndex(const Point2d & point, std::size_t hint = std::numeric_limits<std::size_t>::max()) const;

private:
  void storePath(Path path);
  bool closed_{false};
  std::size_t lap_size_{0};
  Path path_;
};
}  // namespace planning
