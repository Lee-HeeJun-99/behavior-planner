#include "planning_pkg_2025/map/global_path.hpp"

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

namespace planning
{
bool GlobalPath::load(const std::string & file_path, std::string * error)
{
  std::ifstream input(file_path);
  if (!input) {
    if (error) {*error = "cannot open global path: " + file_path;}
    return false;
  }
  Path loaded;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') {continue;}
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    PathPoint point;
    if (!(values >> point.x >> point.y)) {
      if (error) {*error = "invalid global path line " + std::to_string(line_number);}
      return false;
    }
    values >> point.yaw >> point.curvature;
    loaded.push_back(point);
  }
  if (loaded.size() < 2) {
    if (error) {*error = "global path needs at least two points";}
    return false;
  }
  path_ = std::move(loaded);
  return true;
}

void GlobalPath::setPath(Path path) {path_ = std::move(path);}

std::size_t GlobalPath::nearestIndex(const Point2d & point, std::size_t hint) const
{
  if (path_.empty()) {return 0;}
  const std::size_t begin = hint > 100 ? hint - 100 : 0;
  const std::size_t end = std::min(path_.size(), hint + 500);
  std::size_t best = begin;
  double best_squared = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin; i < end; ++i) {
    const double dx = path_[i].x - point.x;
    const double dy = path_[i].y - point.y;
    const double squared = dx * dx + dy * dy;
    if (squared < best_squared) {best_squared = squared; best = i;}
  }
  return best;
}
}  // namespace planning
