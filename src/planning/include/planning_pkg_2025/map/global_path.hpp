#pragma once

#include <cstddef>
#include <string>

#include "planning_pkg_2025/common/planning_types.hpp"

namespace planning
{
class GlobalPath
{
public:
  bool load(const std::string & file_path, std::string * error = nullptr);
  bool load(const std::string & file_path, std::string * error, double smoothing_distance);
  void setPath(Path path);
  const Path & path() const { return path_; }
  bool empty() const { return path_.empty(); }
  std::size_t nearestIndex(const Point2d & point, std::size_t hint = 0) const;

private:
  Path path_;
};
}  // namespace planning
