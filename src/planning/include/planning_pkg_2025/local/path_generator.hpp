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
};

class PathGenerator
{
public:
  explicit PathGenerator(PathGeneratorConfig config = {});
  std::vector<CandidatePath> generate(
    const Path & reference, std::size_t start_index, bool avoidance, double start_lateral_offset) const;

private:
  PathGeneratorConfig config_;
};
}  // namespace planning
