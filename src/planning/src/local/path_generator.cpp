#include "planning_pkg_2025/local/path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace planning
{
namespace
{
double smoothStep(double value)
{
  value = std::clamp(value, 0.0, 1.0);
  return value * value * value * (10.0 + value * (-15.0 + 6.0 * value));
}

void recomputeGeometry(Path & path)
{
  if (path.size() < 3) {return;}
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    path[i].yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
  }
  path.back().yaw = path[path.size() - 2].yaw;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    double delta = std::remainder(path[i + 1].yaw - path[i - 1].yaw, 2.0 * M_PI);
    const double ds = std::hypot(path[i + 1].x - path[i - 1].x, path[i + 1].y - path[i - 1].y);
    path[i].curvature = ds > 1e-6 ? delta / ds : 0.0;
  }
  path.front().curvature = path[1].curvature;
  path.back().curvature = path[path.size() - 2].curvature;
}
}  // namespace

PathGenerator::PathGenerator(PathGeneratorConfig config) : config_(std::move(config)) {}

std::vector<CandidatePath> PathGenerator::generate(
  const Path & reference, std::size_t start_index, bool avoidance, double start_lateral_offset, double return_start_distance) const
{
  std::vector<CandidatePath> candidates;
  if (start_index >= reference.size()) {return candidates;}
  const std::size_t count = std::min(config_.horizon_points, reference.size() - start_index);
  const std::vector<double> offsets = avoidance ? config_.lateral_offsets : std::vector<double>{0.0};
  for (const double target_offset : offsets) {
    CandidatePath candidate;
    candidate.lateral_offset = target_offset;
    candidate.path.reserve(count);
    double travelled = 0.0;
    const std::size_t transition_index = std::min(config_.transition_points, count - 1);
    double transition_distance = 0.0;
    for (std::size_t j = 1; j <= transition_index; ++j) {
      transition_distance += std::hypot(reference[start_index + j].x - reference[start_index + j - 1].x,
        reference[start_index + j].y - reference[start_index + j - 1].y);
    }
    for (std::size_t i = 0; i < count; ++i) {
      if (i > 0) {
        travelled += std::hypot(reference[start_index + i].x - reference[start_index + i - 1].x,
          reference[start_index + i].y - reference[start_index + i - 1].y);
      }
      const auto & ref = reference[start_index + i];
      const double enter = smoothStep(static_cast<double>(i) / std::max<std::size_t>(1, config_.transition_points));
      const std::size_t remaining = count - i - 1;
      const double exit = return_start_distance >= 0.0 ?
        1.0 - smoothStep((travelled - return_start_distance) / std::max(0.1, transition_distance)) :
        smoothStep(static_cast<double>(remaining) / std::max<std::size_t>(1, config_.transition_points));
      const double offset = start_lateral_offset * (1.0 - enter) +
        target_offset * std::min(enter, exit);
      candidate.path.push_back({ref.x - std::sin(ref.yaw) * offset,
        ref.y + std::cos(ref.yaw) * offset, ref.yaw, ref.curvature});
    }
    recomputeGeometry(candidate.path);
    candidates.push_back(std::move(candidate));
  }
  return candidates;
}
}  // namespace planning
