#pragma once

#include "planning_pkg_2025/common/planning_types.hpp"

namespace planning
{
struct CollisionConfig
{
  double vehicle_half_width{0.6};
  double vehicle_front{1.6};
  double vehicle_rear{0.7};
  double safety_margin{0.25};
  double minimum_clearance{0.15};
  double maximum_curvature{0.45};
};

class CollisionChecker
{
public:
  explicit CollisionChecker(CollisionConfig config = {});
  bool validate(CandidatePath & candidate, const BehaviorContext & context) const;

private:
  CollisionConfig config_;
};
}  // namespace planning
