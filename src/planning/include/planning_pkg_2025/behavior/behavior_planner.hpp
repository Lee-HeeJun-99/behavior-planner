#pragma once

#include <chrono>

#include "planning_pkg_2025/common/planning_types.hpp"

namespace planning
{
struct BehaviorConfig
{
  int detection_confirmation_count{3};
  int clear_confirmation_count{5};
  double obstacle_corridor_half_width{0.8};
  double obstacle_lookahead{15.0};
  std::chrono::milliseconds minimum_behavior_duration{500};
  std::chrono::milliseconds stop_hold_duration{2000};
};

class BehaviorPlanner
{
public:
  explicit BehaviorPlanner(BehaviorConfig config = {});
  Behavior update(const BehaviorContext & context, std::chrono::steady_clock::time_point now);
  Behavior current() const {return current_;}

private:
  bool pathBlocked(const BehaviorContext & context) const;
  void transition(Behavior next, std::chrono::steady_clock::time_point now);
  BehaviorConfig config_;
  Behavior current_{Behavior::CRUISE};
  int detection_count_{0};
  int clear_count_{0};
  std::chrono::steady_clock::time_point last_transition_{};
  std::chrono::steady_clock::time_point stop_started_{};
  bool stop_served_{false};
};
}  // namespace planning
