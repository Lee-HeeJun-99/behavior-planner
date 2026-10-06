cmake_minimum_required(VERSION 3.8)
project(behavior_stack_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/debug_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME debug_visualizer_node)

ament_package()
