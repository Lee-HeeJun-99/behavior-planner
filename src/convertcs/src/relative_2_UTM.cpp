#include "relative_2_utm/relative_2_UTM.hpp"

#include <cmath>
#include <functional>
#include <memory>

RelativeToUtm::RelativeToUtm()
: Node("relative_2_utm")
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  declare_parameter<double>("sensor_offset_x", sensor_offset_x_);
  declare_parameter<double>("sensor_offset_y", sensor_offset_y_);
  sensor_offset_x_ = get_parameter("sensor_offset_x").as_double();
  sensor_offset_y_ = get_parameter("sensor_offset_y").as_double();

  position_subscriber_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/Local/utm", qos,
    [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
      vehicle_x_ = message->point.x;
      vehicle_y_ = message->point.y;
      position_received_ = true;
    });
  heading_subscriber_ = create_subscription<std_msgs::msg::Float64>(
    "/Local/heading", qos,
    [this](const std_msgs::msg::Float64::SharedPtr message) {
      heading_ = message->data;
      heading_received_ = true;
    });
  obstacle_subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "/LiDAR/object_cen", qos,
    std::bind(&RelativeToUtm::obstacleCallback, this, std::placeholders::_1));
  obstacle_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/Convert/small_object_UTM", qos);
}

void RelativeToUtm::obstacleCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
  if (!position_received_ || !heading_received_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for localization before obstacle conversion");
    return;
  }

  std_msgs::msg::Float64MultiArray converted;
  for (std::size_t index = 0; index + 1 < message->data.size(); index += 2) {
    const double relative_x = sensor_offset_x_ + message->data[index];
    const double relative_y = sensor_offset_y_ + message->data[index + 1];
    converted.data.push_back(
      vehicle_x_ + relative_x * std::cos(heading_) - relative_y * std::sin(heading_));
    converted.data.push_back(
      vehicle_y_ + relative_x * std::sin(heading_) + relative_y * std::cos(heading_));
  }
  obstacle_publisher_->publish(converted);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelativeToUtm>());
  rclcpp::shutdown();
  return 0;
}
