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
 double stability_lookahead{6.0};
 double stability_margin{0.5};
 double vehicle_front{1.6}, vehicle_rear{0.7};
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
 std::size_t relevantCount() const {return relevant_count_;}
 int confirmationCount() const {return good_count_;}
 long stableMilliseconds(std::chrono::steady_clock::time_point now) const {
  return std::chrono::duration_cast<std::chrono::milliseconds>(now-stable_since_).count();
 }
 Behavior apply(const BehaviorContext & context, Behavior desired, bool feasible,
   bool stopped, bool fresh, std::chrono::steady_clock::time_point now,
   const Path * selected_path = nullptr, bool automatic = true) {
  // Filter only stability matching. Feasibility is still checked against ALL obstacles.
  std::vector<ObstacleContext> relevant;
  for (const auto & obstacle : context.obstacles) {
   if (inRecoveryCorridor(obstacle,context,selected_path)) relevant.push_back(obstacle);
  }
  relevant_count_=relevant.size();
  bool changed = !initialized_ || anchor_.size() != relevant.size();
  if (!changed) {
   std::vector<bool> used(anchor_.size(), false);
   for (const auto & obstacle : relevant) {
    std::size_t best = anchor_.size(); double distance = config_.stability_tolerance;
    for (std::size_t i=0;i<anchor_.size();++i) {
     const double d=std::hypot(anchor_[i].center.x-obstacle.center.x,anchor_[i].center.y-obstacle.center.y);
     if (!used[i] && d<=distance) {distance=d;best=i;}
    }
    if (best==anchor_.size()) {changed=true;break;} used[best]=true;
   }
  }
  if (changed || !fresh || !automatic) {anchor_=relevant;stable_since_=now;initialized_=true;good_count_=0;}
  bool close=false;
  for (const auto & obstacle : context.obstacles) {
   const double dx=obstacle.center.x-context.vehicle.position.x;
   const double dy=obstacle.center.y-context.vehicle.position.y;
   const double along=std::cos(context.vehicle.yaw)*dx+std::sin(context.vehicle.yaw)*dy;
   const double lateral=-std::sin(context.vehicle.yaw)*dx+std::cos(context.vehicle.yaw)*dy;
   if (along>=0 && along<=config_.close_distance && std::abs(lateral)<=config_.corridor_half_width+obstacle.radius) {close=true;}
  }
  const bool unsafe=!automatic || !feasible || context.emergency_stop || !context.vehicle.localization_valid || !fresh;
  if (!holding_ && (unsafe || (close && (!low_speed_ || changed)))) {
   holding_=true;low_speed_=false;hold_since_=now;good_count_=0;
  }
  if (holding_) {
   const bool ready=!unsafe && stopped && now-hold_since_>=config_.hold_duration &&
     now-stable_since_>=config_.stable_duration;
   good_count_=ready ? good_count_+1 : 0;
   if (good_count_<std::max(1,config_.feasible_confirmation_count)) return Behavior::EMERGENCY_STOP;
   holding_=false;low_speed_=true;low_speed_since_=now;good_count_=0;
  }
  if (low_speed_ && desired==Behavior::CRUISE && relevant.empty() &&
    now-low_speed_since_>=config_.stable_duration && now-stable_since_>=config_.stable_duration) low_speed_=false;
  return desired;
 }
 private:
 bool inRecoveryCorridor(const ObstacleContext & obstacle, const BehaviorContext & context,
   const Path * path) const {
  const double width=config_.corridor_half_width+obstacle.radius+config_.stability_margin;
  const auto inside=[&](double x,double y,double yaw,double forward) {
   const double dx=obstacle.center.x-x,dy=obstacle.center.y-y;
   const double along=std::cos(yaw)*dx+std::sin(yaw)*dy;
   const double lateral=-std::sin(yaw)*dx+std::cos(yaw)*dy;
   return along>=-config_.vehicle_rear-obstacle.radius-config_.stability_margin &&
     along<=forward+obstacle.radius+config_.stability_margin && std::abs(lateral)<=width;
  };
  // Always retain the immediate body/front corridor, including when no path exists.
  if (inside(context.vehicle.position.x,context.vehicle.position.y,context.vehicle.yaw,
    config_.close_distance)) return true;
  if (path && !path->empty()) {
   double arc=0.0;
   for (std::size_t i=0;i<path->size();++i) {
    if (i>0) arc+=std::hypot((*path)[i].x-(*path)[i-1].x,(*path)[i].y-(*path)[i-1].y);
    if (arc>config_.stability_lookahead) break;
    const auto & p=(*path)[i];
    if (inside(p.x,p.y,p.yaw,config_.vehicle_front)) return true;
   }
   return false;
  }
  return inside(context.vehicle.position.x,context.vehicle.position.y,context.vehicle.yaw,
    config_.stability_lookahead+config_.vehicle_front);
 }
 std::size_t relevant_count_{0};
 RecoveryConfig config_;bool holding_{false},low_speed_{false},initialized_{false};int good_count_{0};
 std::vector<ObstacleContext> anchor_;
 std::chrono::steady_clock::time_point hold_since_{},stable_since_{},low_speed_since_{};
};
}
