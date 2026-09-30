#include "LiDAR_small_static.cpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto small_static = std::make_shared<LiDAR_small_static>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(small_static);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
