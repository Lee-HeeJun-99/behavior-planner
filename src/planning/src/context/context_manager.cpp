#include "planning_pkg_2025/context/context_manager.hpp"

#include <algorithm>
#include <cmath>

namespace planning
{
namespace {
double distance(const Point2d & a, const Point2d & b) {return std::hypot(a.x - b.x, a.y - b.y);}
Point2d position(const PathPoint & p) {return {p.x,p.y};}
bool segmentOverlapsRoad(const Point2d & point, const Point2d & a, const Point2d & b,
  const ContextConfig & config, bool forward=true) {
  const double dx=b.x-a.x,dy=b.y-a.y;
  const double length2=dx*dx+dy*dy;
  const double cap=config.obstacle_radius+config.obstacle_path_margin;
  if(length2<1e-12)return distance(point,a)<=cap;
  const double t=((point.x-a.x)*dx+(point.y-a.y)*dy)/length2;
  if(t<0)return distance(point,a)<=cap;
  if(t>1)return distance(point,b)<=cap;
  const double lateral=(forward ? 1.0 : -1.0)*
    (dx*(point.y-a.y)-dy*(point.x-a.x))/std::sqrt(length2);
  return lateral<=config.road_left_bound+config.obstacle_path_margin &&
    lateral>=-config.road_right_bound-config.obstacle_path_margin;
}
// Restrict matching to the active route branch, not all nearby map segments.
bool overlapsActivePath(const Point2d & point, const Path & path, std::size_t index,
  const ContextConfig & config) {
  if(index>=path.size())return false;
  const double radius=config.obstacle_radius+config.obstacle_path_margin;
  const auto scan=[&](bool forward,double limit) {
    std::size_t current=index;double arc=0.0;
    while(arc<limit && (forward ? current+1<path.size() : current>0)) {
      const std::size_t next=forward ? current+1 : current-1;
      const auto a=position(path[current]);auto b=position(path[next]);const double length=distance(a,b);
      if(length>limit-arc) {const double t=(limit-arc)/length;b={a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)};}
      if(segmentOverlapsRoad(point,a,b,config,forward))return true;
      arc+=length;current=next;
    }
    // At the route seam, retain just-passed detections in a short rear corridor.
    if(!forward && current==0 && arc<limit) {
      const auto a=position(path.front());const double rear=limit-arc;
      const Point2d b{a.x-rear*std::cos(path.front().yaw),a.y-rear*std::sin(path.front().yaw)};
      if(segmentOverlapsRoad(point,a,b,config,false))return true;
    }
    return false;
  };
  return distance(point,position(path[index]))<=radius ||
    scan(true,config.obstacle_path_lookahead) || scan(false,config.obstacle_path_lookbehind);
}
}  // namespace

ContextManager::ContextManager(ContextConfig config) : config_(config)
{
  speed_.active_limit_kph = (config_.default_speed_limit_kph == 0 || isSupportedSpeedLimit(config_.default_speed_limit_kph)) ?
    config_.default_speed_limit_kph : 20;
  speed_.target_velocity = config_.default_target_velocity >= 0.0 ?
    config_.default_target_velocity : targetVelocityForLimit(speed_.active_limit_kph);
  speed_.avoid_max_velocity = std::max(0.0, config_.avoid_max_velocity);
}

void ContextManager::updateVehicle(const Point2d & position, double yaw)
{
  vehicle_.position = position;
  vehicle_.yaw = yaw;
  vehicle_.localization_valid = std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(yaw);
}

void ContextManager::updateObstacles(const std::vector<Point2d> & points) {raw_obstacles_ = points;}

bool ContextManager::isSupportedSpeedLimit(int limit_kph) const
{
  return limit_kph == 20 || limit_kph == 50;
}

double ContextManager::targetVelocityForLimit(int limit_kph) const
{
  if (limit_kph == 50) {return config_.speed_limit_50_target;}
  return config_.speed_limit_20_target;
}

void ContextManager::updateSpeedLimitDetection(int detected_limit_kph)
{
  speed_.detected_limit_kph = detected_limit_kph;
  speed_.detection_valid = isSupportedSpeedLimit(detected_limit_kph);
  if (!speed_.detection_valid) {
    speed_candidate_kph_ = 0;
    speed_candidate_count_ = 0;
    return;
  }
  if (speed_candidate_kph_ == detected_limit_kph) {
    ++speed_candidate_count_;
  } else {
    speed_candidate_kph_ = detected_limit_kph;
    speed_candidate_count_ = 1;
  }
  if (speed_candidate_count_ >= std::max(1, config_.speed_sign_confirmation_count)) {
    speed_.active_limit_kph = detected_limit_kph;
    speed_.target_velocity = targetVelocityForLimit(detected_limit_kph);
  }
}

BehaviorContext ContextManager::build(const Path & global_path, std::size_t nearest_index) const
{
  BehaviorContext context;
  context.vehicle = vehicle_;
  context.path = {&global_path, nearest_index};
  context.speed = speed_;
  context.free_space = {config_.road_left_bound, config_.road_right_bound};
  context.emergency_stop = emergency_stop_;
  for (const auto & point : raw_obstacles_) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      distance(vehicle_.position, point) > config_.obstacle_relevance_distance) {continue;}
    if (config_.filter_obstacles_by_path) {
      const double dx=point.x-vehicle_.position.x,dy=point.y-vehicle_.position.y;
      const double lateral=-std::sin(vehicle_.yaw)*dx+std::cos(vehicle_.yaw)*dy;
      if(lateral>config_.road_left_bound || lateral< -config_.road_right_bound ||
        !overlapsActivePath(point,global_path,nearest_index,config_)) {continue;}
    }
    context.obstacles.push_back({point, config_.obstacle_radius});
  }
  return context;
}
}  // namespace planning
