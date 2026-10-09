#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "planning_pkg_2025/behavior/behavior_planner.hpp"
#include "planning_pkg_2025/context/context_manager.hpp"
#include "planning_pkg_2025/local/local_planner.hpp"
#include "planning_pkg_2025/map/global_path.hpp"

namespace planning
{
class PlanningNode : public rclcpp::Node
{
public:
  PlanningNode() : Node("planning_node")
  {
    declareParameters();
    configureModules();
    loadGlobalPath();
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    position_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/Local/utm", qos, [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        position_ = {msg->point.x, msg->point.y}; position_received_ = true;});
    heading_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/Local/heading", qos, [this](std_msgs::msg::Float64::SharedPtr msg) {
        heading_ = msg->data; heading_received_ = true;});
    small_obstacles_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/Convert/small_object_UTM", qos, [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        small_obstacles_ = decodePoints(msg->data);});
    big_obstacles_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/Convert/big_object_UTM", qos, [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        big_obstacles_ = decodePoints(msg->data);});
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/LiDAR/dynamic_stop", qos, [this](std_msgs::msg::Bool::SharedPtr msg) {emergency_stop_ = msg->data;});
    speed_limit_sub_ = create_subscription<std_msgs::msg::Int16>(
      "/Perception/speed_limit", qos, [this](std_msgs::msg::Int16::SharedPtr msg) {
        context_manager_.updateSpeedLimitDetection(msg->data);});

    path_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/local_path", qos);
    yaw_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/path_yaw", qos);
    curvature_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/curvature", qos);
    target_velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/Planning/target_velocity", qos);
    mission_pub_ = create_publisher<std_msgs::msg::Int16>("/Planning/mission", qos);
    behavior_pub_ = create_publisher<std_msgs::msg::String>("/Planning/behavior", qos);
    debug_path_pub_ = create_publisher<nav_msgs::msg::Path>("/Planning/debug/local_path", qos);
    candidates_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/debug/candidates", qos);
    global_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/Planning/debug/global_path", rclcpp::QoS(1).reliable().transient_local());
    candidate_paths_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/Planning/debug/candidate_paths", qos);
    publishGlobalPath();
    timer_ = create_wall_timer(std::chrono::milliseconds(get_parameter("planning_period_ms").as_int()),
      std::bind(&PlanningNode::onTimer, this));
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("global_path_file", "map/map_final_0921/0-0.txt");
    declare_parameter<int>("planning_period_ms", 50);
    declare_parameter<std::vector<double>>("lateral_offsets", {-1.2, -0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9, 1.2});
    declare_parameter<int>("transition_points", 120);
    declare_parameter<double>("avoidance_start_distance", 10.0);
    declare_parameter<double>("obstacle_pass_margin", 3.0);
    declare_parameter<double>("road_left_bound", 1.5);
    declare_parameter<double>("road_right_bound", 1.5);
    declare_parameter<double>("vehicle_half_width", 0.6);
    declare_parameter<double>("vehicle_front", 1.6);
    declare_parameter<double>("vehicle_rear", 0.7);
    declare_parameter<double>("safety_margin", 0.25);
    declare_parameter<double>("maximum_curvature", 0.45);
    declare_parameter<int>("detection_confirmation_count", 3);
    declare_parameter<int>("clear_confirmation_count", 5);
    declare_parameter<int>("minimum_behavior_duration_ms", 500);
    declare_parameter<int>("default_speed_limit_kph", 20);
    declare_parameter<int>("speed_sign_confirmation_count", 3);
    declare_parameter<double>("speed_limit_20_target", 2.5);
    declare_parameter<double>("speed_limit_50_target", 4.5);
    declare_parameter<double>("avoid_max_velocity", 2.5);
  }

  void configureModules()
  {
    ContextConfig context_config;
    context_config.road_left_bound = get_parameter("road_left_bound").as_double();
    context_config.road_right_bound = get_parameter("road_right_bound").as_double();
    context_config.default_speed_limit_kph = get_parameter("default_speed_limit_kph").as_int();
    context_config.speed_sign_confirmation_count = get_parameter("speed_sign_confirmation_count").as_int();
    context_config.speed_limit_20_target = get_parameter("speed_limit_20_target").as_double();
    context_config.speed_limit_50_target = get_parameter("speed_limit_50_target").as_double();
    context_config.avoid_max_velocity = get_parameter("avoid_max_velocity").as_double();
    if (context_config.default_speed_limit_kph != 20 &&
      context_config.default_speed_limit_kph != 50)
    {
      RCLCPP_WARN(
        get_logger(), "Unsupported default_speed_limit_kph=%d; falling back to 20",
        context_config.default_speed_limit_kph);
    }
    context_manager_ = ContextManager(context_config);

    BehaviorConfig behavior_config;
    behavior_config.obstacle_lookahead = get_parameter("avoidance_start_distance").as_double();
    behavior_config.obstacle_pass_margin = get_parameter("obstacle_pass_margin").as_double();
    behavior_config.detection_confirmation_count = get_parameter("detection_confirmation_count").as_int();
    behavior_config.clear_confirmation_count = get_parameter("clear_confirmation_count").as_int();
    behavior_config.minimum_behavior_duration =
      std::chrono::milliseconds(get_parameter("minimum_behavior_duration_ms").as_int());
    behavior_planner_ = BehaviorPlanner(behavior_config);

    PathGeneratorConfig generator_config;
    generator_config.avoidance_start_distance = get_parameter("avoidance_start_distance").as_double();
    generator_config.obstacle_pass_margin = get_parameter("obstacle_pass_margin").as_double();
    generator_config.lateral_offsets = get_parameter("lateral_offsets").as_double_array();
    generator_config.transition_points = static_cast<std::size_t>(get_parameter("transition_points").as_int());
    CollisionConfig collision_config;
    collision_config.vehicle_half_width = get_parameter("vehicle_half_width").as_double();
    collision_config.vehicle_front = get_parameter("vehicle_front").as_double();
    collision_config.vehicle_rear = get_parameter("vehicle_rear").as_double();
    collision_config.safety_margin = get_parameter("safety_margin").as_double();
    collision_config.maximum_curvature = get_parameter("maximum_curvature").as_double();
    local_planner_ = LocalPlanner(PathGenerator(generator_config), CollisionChecker(collision_config), CostEvaluator{});
  }

  void loadGlobalPath()
  {
    std::string file = get_parameter("global_path_file").as_string();
    if (!file.empty() && file.front() != '/') {
      file = ament_index_cpp::get_package_share_directory("planning_pkg_2025") + "/" + file;
    }
    std::string error;
    if (!global_path_.load(file, &error)) {throw std::runtime_error(error);}
    RCLCPP_INFO(get_logger(), "Loaded %zu preferred-path points from %s", global_path_.path().size(), file.c_str());
  }

  static std::vector<Point2d> decodePoints(const std::vector<double> & data)
  {
    std::vector<Point2d> result;
    for (std::size_t i = 0; i + 1 < data.size(); i += 2) {result.push_back({data[i], data[i + 1]});}
    return result;
  }

  void publishGlobalPath()
  {
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = "map";
    for (const auto & point : global_path_.path()) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.z = std::sin(point.yaw * 0.5);
      pose.pose.orientation.w = std::cos(point.yaw * 0.5);
      message.poses.push_back(pose);
    }
    global_path_pub_->publish(message);
  }

  void publishCandidatePaths(const LocalPlan & plan)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int marker_id = 0;
    for (const auto & candidate : plan.candidates) {
      visualization_msgs::msg::Marker line;
      line.header.stamp = now();
      line.header.frame_id = "map";
      line.ns = "candidate_paths";
      line.id = marker_id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.06;
      line.color.a = candidate.valid ? 0.9 : 0.35;
      const bool selected = plan.feasible && candidate.valid &&
        candidate.lateral_offset == plan.selected_offset;
      if (selected) {
        line.color.g = 1.0;
      } else if (candidate.valid) {
        line.color.b = 1.0;
        line.color.g = 0.6;
      } else {
        line.color.r = 1.0;
      }
      for (const auto & point : candidate.path) {
        geometry_msgs::msg::Point geometry_point;
        geometry_point.x = point.x;
        geometry_point.y = point.y;
        geometry_point.z = 0.05;
        line.points.push_back(geometry_point);
      }
      markers.markers.push_back(line);

      if (!candidate.path.empty()) {
        visualization_msgs::msg::Marker label;
        label.header = line.header;
        label.ns = "candidate_labels";
        label.id = marker_id++;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x = candidate.path.front().x;
        label.pose.position.y = candidate.path.front().y;
        label.pose.position.z = 0.6;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.28;
        label.color.a = 1.0;
        label.color.r = selected ? 0.0 : 1.0;
        label.color.g = selected ? 1.0 : 1.0;
        label.color.b = selected ? 0.0 : 1.0;
        label.text = (selected ? "selected " : "") +
          std::string("offset=") + std::to_string(candidate.lateral_offset) +
          (candidate.valid ? " valid cost=" + std::to_string(candidate.cost) +
          " clearance=" + std::to_string(candidate.minimum_clearance) :
          " invalid " + candidate.invalid_reason);
        markers.markers.push_back(label);
      }
    }
    candidate_paths_pub_->publish(markers);
  }

  void onTimer()
  {
    if (!position_received_ || !heading_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for localization"); return;
    }
    nearest_index_ = global_path_.nearestIndex(position_, nearest_index_);
    context_manager_.updateVehicle(position_, heading_);
    std::vector<Point2d> obstacles = small_obstacles_;
    obstacles.insert(obstacles.end(), big_obstacles_.begin(), big_obstacles_.end());
    context_manager_.updateObstacles(obstacles);
    context_manager_.setEmergencyStop(emergency_stop_);
    const auto context = context_manager_.build(global_path_.path(), nearest_index_);
    Behavior behavior = behavior_planner_.update(context, std::chrono::steady_clock::now());
    LocalPlan plan = local_planner_.plan(behavior, context);
    if (!plan.feasible && behavior == Behavior::CRUISE && !context.obstacles.empty()) {
      plan = local_planner_.plan(Behavior::AVOID, context);
    }
    if (!plan.feasible) {
      behavior = Behavior::EMERGENCY_STOP;
    }
    publish(plan, behavior, context);
  }

  void publish(const LocalPlan & plan, Behavior behavior, const BehaviorContext & context)
  {
    std_msgs::msg::Float64MultiArray positions;
    std_msgs::msg::Float64MultiArray yaws;
    std_msgs::msg::Float64MultiArray curvatures;
    nav_msgs::msg::Path debug;
    debug.header.stamp = now(); debug.header.frame_id = "map";
    for (const auto & point : plan.path) {
      positions.data.insert(positions.data.end(), {point.x, point.y});
      yaws.data.push_back(point.yaw); curvatures.data.push_back(point.curvature);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = debug.header; pose.pose.position.x = point.x; pose.pose.position.y = point.y;
      pose.pose.orientation.z = std::sin(point.yaw * 0.5); pose.pose.orientation.w = std::cos(point.yaw * 0.5);
      debug.poses.push_back(pose);
    }
    std_msgs::msg::Float64MultiArray candidates;
    for (const auto & candidate : plan.candidates) {
      candidates.data.push_back(candidate.lateral_offset);
      candidates.data.push_back(candidate.valid ? candidate.cost : -1.0);
      candidates.data.push_back(candidate.minimum_clearance);
    }
    std_msgs::msg::Float64 velocity;
    velocity.data = calculateTargetVelocity(behavior, context);
    std_msgs::msg::Int16 mission;
    mission.data = behavior == Behavior::AVOID ? 14 :
      (behavior == Behavior::CRUISE ? 0 : 33);
    std_msgs::msg::String behavior_message; behavior_message.data = toString(behavior);
    path_pub_->publish(positions); yaw_pub_->publish(yaws); curvature_pub_->publish(curvatures);
    target_velocity_pub_->publish(velocity); mission_pub_->publish(mission);
    behavior_pub_->publish(behavior_message); debug_path_pub_->publish(debug); candidates_pub_->publish(candidates);
    publishCandidatePaths(plan);
  }

  GlobalPath global_path_;
  ContextManager context_manager_;
  BehaviorPlanner behavior_planner_;
  LocalPlanner local_planner_;
  Point2d position_;
  double heading_{0.0};
  bool position_received_{false};
  bool heading_received_{false};
  bool emergency_stop_{false};
  std::size_t nearest_index_{0};
  std::vector<Point2d> small_obstacles_;
  std::vector<Point2d> big_obstacles_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr small_obstacles_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr big_obstacles_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr speed_limit_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr path_pub_, yaw_pub_, curvature_pub_, candidates_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_paths_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planning::PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
