#pragma once

#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class RelativeToUtm : public rclcpp::Node
{
public:
  RelativeToUtm();

private:
  void obstacleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr message);

  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  double heading_{0.0};
  bool position_received_{false};
  bool heading_received_{false};
  double sensor_offset_x_{0.35};
  double sensor_offset_y_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_publisher_;
};
