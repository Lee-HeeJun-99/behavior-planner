#pragma once
#include <chrono>
#include <cmath>
#include <algorithm>
#include "planning_pkg_2025/common/planning_types.hpp"
namespace planning {
struct RecoveryConfig {
 double close_distance{3.0};
 double corridor_half_width{0.85};
 double stability_tolerance{0.35};
 double recovery_velocity{0.5 / 3.6};
 int feasible_confirmation_count{10};
 std::chrono::milliseconds hold_duration{1000}, stable_duration{1000};
};
class RecoveryGuard {
 public:
 explicit RecoveryGuard(RecoveryConfig config = {}) : config_(config) {}
 bool lowSpeed() const {return low_speed_;}
 bool holding() const {return holding_;}
 double speedCap() const {return config_.recovery_velocity;}
 Behavior apply(const BehaviorContext & context, Behavior desired, bool feasible,
   bool stopped, bool fresh, std::chrono::steady_clock::time_point now) {
  bool changed = !initialized_ || anchor_.size() != context.obstacles.size();
  if (!changed) {
   std::vector<bool> used(anchor_.size(), false);
   for (const auto & obstacle : context.obstacles) {
    std::size_t best = anchor_.size(); double distance = config_.stability_tolerance;
    for (std::size_t i=0;i<anchor_.size();++i) {
     const double d=std::hypot(anchor_[i].center.x-obstacle.center.x,anchor_[i].center.y-obstacle.center.y);
     if (!used[i] && d<=distance) {distance=d;best=i;}
    }
    if (best==anchor_.size()) {changed=true;break;} used[best]=true;
   }
  }
  if (changed || !fresh) {anchor_=context.obstacles;stable_since_=now;initialized_=true;good_count_=0;}
  bool close=false;
  for (const auto & obstacle : context.obstacles) {
   const double dx=obstacle.center.x-context.vehicle.position.x;
   const double dy=obstacle.center.y-context.vehicle.position.y;
   const double along=std::cos(context.vehicle.yaw)*dx+std::sin(context.vehicle.yaw)*dy;
   const double lateral=-std::sin(context.vehicle.yaw)*dx+std::cos(context.vehicle.yaw)*dy;
   if (along>=0 && along<=config_.close_distance && std::abs(lateral)<=config_.corridor_half_width+obstacle.radius) {close=true;}
  }
  const bool unsafe=!feasible || context.emergency_stop || !context.vehicle.localization_valid || !fresh;
  if (!holding_ && (unsafe || (close && (!low_speed_ || changed)))) {
   holding_=true;low_speed_=false;hold_since_=now;good_count_=0;
  }
  if (holding_) {
   const bool ready=!unsafe && stopped && now-hold_since_>=config_.hold_duration &&
     now-stable_since_>=config_.stable_duration;
   good_count_=ready ? good_count_+1 : 0;
   if (good_count_<std::max(1,config_.feasible_confirmation_count)) return Behavior::EMERGENCY_STOP;
   holding_=false;low_speed_=true;good_count_=0;
  }
  if (low_speed_ && desired==Behavior::CRUISE && context.obstacles.empty()) low_speed_=false;
  return desired;
 }
 private:
 RecoveryConfig config_;bool holding_{false},low_speed_{false},initialized_{false};int good_count_{0};
 std::vector<ObstacleContext> anchor_;
 std::chrono::steady_clock::time_point hold_since_{},stable_since_{};
};
}
