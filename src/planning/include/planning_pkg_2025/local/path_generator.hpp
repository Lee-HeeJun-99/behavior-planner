#pragma once

#include <vector>

#include "planning_pkg_2025/common/planning_types.hpp"

namespace planning
{
struct PathGeneratorConfig
{
  std::vector<double> lateral_offsets{-1.2, -0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9, 1.2};
  std::size_t horizon_points{300};
  std::size_t transition_points{80};
  double avoidance_start_distance{10.0};
  double obstacle_pass_margin{3.0};
  double preferred_avoidance_offset{0.0};
};

class PathGenerator
{
public:
  explicit PathGenerator(PathGeneratorConfig config = {});
  const PathGeneratorConfig & config() const {return config_;}
  std::vector<CandidatePath> generate(
    const Path & reference, std::size_t start_index, bool avoidance, double start_lateral_offset,
    double return_start_distance = -1.0) const;

  // Try other merge lengths only after the configured profile fails.
  std::vector<CandidatePath> generateTransitionVariants(
    const Path & reference, std::size_t start_index, bool avoidance, double start_lateral_offset,
    double return_start_distance = -1.0) const;

  // Multi-stage lateral profiles for corners where a constant offset is infeasible.
  std::vector<CandidatePath> generateCornerTransitions(
    const Path & reference, std::size_t start_index, double start_lateral_offset) const;

private:
  PathGeneratorConfig config_;
};
}  // namespace planning
