cmake_minimum_required(VERSION 3.12)
project(planning_pkg_2025)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
add_library(planning_core
  src/map/global_path.cpp
  src/context/context_manager.cpp
  src/behavior/behavior_planner.cpp
  src/local/path_generator.cpp
  src/local/collision_checker.cpp
  src/local/cost_evaluator.cpp
  src/local/local_planner.cpp
)
target_include_directories(planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(planning_node src/nodes/planning_node.cpp)
target_link_libraries(planning_node planning_core)
ament_target_dependencies(planning_node ament_index_cpp geometry_msgs nav_msgs rclcpp std_msgs)

install(TARGETS planning_core planning_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch map DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(planning_core_test test/planning_core_test.cpp)
  target_link_libraries(planning_core_test planning_core)
  target_include_directories(planning_core_test PRIVATE include)
  set_tests_properties(planning_core_test PROPERTIES
    ENVIRONMENT "PLANNING_TEST_MAP=${CMAKE_CURRENT_SOURCE_DIR}/map/map_final_0921/0-0.txt"
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(planning_core)
ament_package()
