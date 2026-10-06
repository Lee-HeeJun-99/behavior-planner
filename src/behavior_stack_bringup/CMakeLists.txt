cmake_minimum_required(VERSION 3.8)
project(behavior_stack_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/debug_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME debug_visualizer_node)
install(PROGRAMS scripts/mock_debug_input_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME mock_debug_input_node)
install(PROGRAMS scripts/mock_camera_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME mock_camera_node)
install(FILES ../planning/test/integration_planning.yaml
  DESTINATION share/${PROJECT_NAME}/config RENAME mock_planning.yaml)
install(FILES ../planning/test/ros2_local_planner_integration.py
  DESTINATION lib/${PROJECT_NAME} RENAME planning_test_scenarios.py)

ament_package()
