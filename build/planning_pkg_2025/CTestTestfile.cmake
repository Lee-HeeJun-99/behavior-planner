# CMake generated Testfile for 
# Source directory: /home/ubuntu/lhj_behavior_stage/src/planning
# Build directory: /home/ubuntu/lhj_behavior_stage/build/planning_pkg_2025
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(planning_core_test "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ubuntu/lhj_behavior_stage/build/planning_pkg_2025/test_results/planning_pkg_2025/planning_core_test.gtest.xml" "--package-name" "planning_pkg_2025" "--output-file" "/home/ubuntu/lhj_behavior_stage/build/planning_pkg_2025/ament_cmake_gtest/planning_core_test.txt" "--command" "/home/ubuntu/lhj_behavior_stage/build/planning_pkg_2025/planning_core_test" "--gtest_output=xml:/home/ubuntu/lhj_behavior_stage/build/planning_pkg_2025/test_results/planning_pkg_2025/planning_core_test.gtest.xml")
set_tests_properties(planning_core_test PROPERTIES  ENVIRONMENT "PLANNING_TEST_MAP=/home/ubuntu/lhj_behavior_stage/src/planning/map/map_final_0921/0-0.txt" LABELS "gtest" REQUIRED_FILES "/home/ubuntu/lhj_behavior_stage/build/planning_pkg_2025/planning_core_test" TIMEOUT "60" WORKING_DIRECTORY "/home/ubuntu/lhj_behavior_stage/build/planning_pkg_2025" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/ubuntu/lhj_behavior_stage/src/planning/CMakeLists.txt;45;ament_add_gtest;/home/ubuntu/lhj_behavior_stage/src/planning/CMakeLists.txt;0;")
subdirs("gtest")
