#include "planning_pkg_2025/map/global_path.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

namespace planning
{
bool GlobalPath::load(const std::string & file_path, std::string * error)
{
  return load(file_path, error, 0.75);
}

bool GlobalPath::load(const std::string & file_path, std::string * error, double smoothing_distance)
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
  // Smooth short survey/join discontinuities using distance-weighted coordinates.
  // Preserve endpoints; never rewrite the user's map file.
  if (smoothing_distance > 0.0) {
    const Path original = loaded;
    std::vector<double> arc(loaded.size(), 0.0);
    for (std::size_t i = 1; i < loaded.size(); ++i) {
      arc[i] = arc[i - 1] + std::hypot(original[i].x - original[i - 1].x, original[i].y - original[i - 1].y);
    }
    for (std::size_t i = 1; i + 1 < loaded.size(); ++i) {
      const double sigma = std::min(smoothing_distance, std::min(arc[i], arc.back() - arc[i]) / 3.0);
      if (sigma < 1e-6) {continue;}
      auto begin = std::lower_bound(arc.begin(), arc.end(), arc[i] - 3.0 * sigma);
      auto end = std::upper_bound(arc.begin(), arc.end(), arc[i] + 3.0 * sigma);
      double x = 0.0, y = 0.0, weight = 0.0;
      for (auto j = static_cast<std::size_t>(begin - arc.begin()); j < static_cast<std::size_t>(end - arc.begin()); ++j) {
        const double d = (arc[j] - arc[i]) / sigma;
        const double w = std::exp(-0.5 * d * d);
        x += w * original[j].x; y += w * original[j].y; weight += w;
      }
      const double dx = x / weight - original[i].x;
      const double dy = y / weight - original[i].y;
      // Bound changes by 30 cm, independent of the requested smoothing window.
      const double shift = std::hypot(dx, dy);
      const double scale = shift > 0.3 ? 0.3 / shift : 1.0;
      loaded[i].x = original[i].x + dx * scale;
      loaded[i].y = original[i].y + dy * scale;
    }
  }
  for (std::size_t i = 0; i < loaded.size(); ++i) {
    const auto & before = loaded[i > 0 ? i - 1 : i];
    const auto & after = loaded[i + 1 < loaded.size() ? i + 1 : i];
    loaded[i].yaw = std::atan2(after.y - before.y, after.x - before.x);
  }
  for (std::size_t i = 1; i + 1 < loaded.size(); ++i) {
    const double ds = std::hypot(loaded[i + 1].x - loaded[i - 1].x, loaded[i + 1].y - loaded[i - 1].y);
    loaded[i].curvature = ds > 1e-6 ? std::remainder(loaded[i + 1].yaw - loaded[i - 1].yaw, 2.0 * M_PI) / ds : 0.0;
  }
  loaded.front().curvature = loaded[1].curvature;
  loaded.back().curvature = loaded[loaded.size() - 2].curvature;
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
