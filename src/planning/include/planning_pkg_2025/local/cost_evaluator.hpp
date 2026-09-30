#pragma once

#include "planning_pkg_2025/common/planning_types.hpp"

namespace planning
{
struct CostConfig
{
  double deviation_weight{4.0};
  double curvature_weight{1.0};
  double smoothness_weight{1.0};
  double clearance_weight{2.0};
};

class CostEvaluator
{
public:
  explicit CostEvaluator(CostConfig config = {});
  double evaluate(CandidatePath & candidate) const;

private:
  CostConfig config_;
};
}  // namespace planning
