#ifndef MISSION_HPP
#define MISSION_HPP

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "data_header/qos_utils.hpp"

class Mission : public rclcpp::Node
{
public:
  explicit Mission(const std::string & node_name)
  : Node(node_name)
  {
    pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", lidar::best_effort_qos(),
      std::bind(&Mission::callback, this, std::placeholders::_1));
  }

private:
  virtual void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
};

#endif  // MISSION_HPP
