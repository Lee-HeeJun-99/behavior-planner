#include "LiDAR_small_static.cpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto detector = std::make_shared<LidarObstacleDetector>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(detector);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
