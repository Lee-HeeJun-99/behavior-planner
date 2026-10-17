#pragma once
#include <chrono>
#include <cmath>
#include <algorithm>
namespace planning {
struct AutoEntryConfig {
 double max_route_distance{0.5};
 double max_heading_error{15.0*3.14159265358979323846/180.0};
 double restart_velocity{0.5/3.6};
 int confirmation_count{10};
 std::chrono::milliseconds stable_duration{1000},low_speed_duration{1000};
};
class AutoEntryGuard {
 public:
 explicit AutoEntryGuard(AutoEntryConfig config = {}) : config_(config) {}
 bool automatic() const {return mode_==1;}
 bool holding() const {return holding_;}
 int confirmationCount() const {return count_;}
 double speedCap() const {return config_.restart_velocity;}
 bool lowSpeed(std::chrono::steady_clock::time_point now) const {
  return automatic() && !holding_ && now-approved_at_<config_.low_speed_duration;
 }
 void observeMode(int mode,std::chrono::steady_clock::time_point now) {
  if(mode!=0 && mode!=1)mode=-1;
  if(mode!=mode_){mode_=mode;holding_=true;count_=0;ready_since_=now;}
 }
 bool update(bool fresh,bool stopped,bool feasible,double distance,double heading_error,
   std::chrono::steady_clock::time_point now) {
  if(!automatic() || !fresh){holding_=true;count_=0;ready_since_=now;return false;}
  // Qualification is for entry, not a lateral-error stop during normal avoidance.
  if(!holding_)return true;
  const bool valid=stopped && feasible && std::isfinite(distance) && std::isfinite(heading_error) &&
   distance>=0.0 && distance<=config_.max_route_distance &&
   std::abs(heading_error)<=config_.max_heading_error;
  if(!valid){count_=0;ready_since_=now;return false;}
  if(now-ready_since_<config_.stable_duration){count_=0;return false;}
  if(++count_<std::max(1,config_.confirmation_count))return false;
  holding_=false;approved_at_=now;return true;
 }
 private:
 AutoEntryConfig config_;int mode_{-1};bool holding_{true};int count_{0};
 std::chrono::steady_clock::time_point ready_since_{},approved_at_{};
};
}
