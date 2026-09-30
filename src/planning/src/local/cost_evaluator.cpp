#include "planning_pkg_2025/local/cost_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace planning
{
CostEvaluator::CostEvaluator(CostConfig config) : config_(config) {}

double CostEvaluator::evaluate(CandidatePath & candidate) const
{
  if (!candidate.valid) {return candidate.cost;}
  double curvature = 0.0;
  double smoothness = 0.0;
  for (std::size_t i = 0; i < candidate.path.size(); ++i) {
    curvature += candidate.path[i].curvature * candidate.path[i].curvature;
    if (i > 0) {
      const double delta = candidate.path[i].curvature - candidate.path[i - 1].curvature;
      smoothness += delta * delta;
    }
  }
  const double count = std::max<std::size_t>(1, candidate.path.size());
  const double clearance = std::isfinite(candidate.minimum_clearance) ?
    1.0 / std::max(0.05, candidate.minimum_clearance) : 0.0;
  candidate.cost = config_.deviation_weight * candidate.lateral_offset * candidate.lateral_offset +
    config_.curvature_weight * curvature / count +
    config_.smoothness_weight * smoothness / count + config_.clearance_weight * clearance;
  return candidate.cost;
}
}  // namespace planning
