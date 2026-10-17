cmake_minimum_required(VERSION 3.8)
project(arm_joint_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(arm_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ---- pure C++ library (no ROS): trajectory maths + serial port --------------
add_library(${PROJECT_NAME}_core
  src/trapezoid_profile.cpp
  src/serial_link.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_core PUBLIC arm_common::arm_common)

# ---- nodes -----------------------------------------------------------------
add_executable(joint_bridge
  src/joint_bridge_node.cpp
  src/joint_bridge_main.cpp)
target_link_libraries(joint_bridge ${PROJECT_NAME}_core)
ament_target_dependencies(joint_bridge rclcpp sensor_msgs trajectory_msgs diagnostic_msgs std_srvs)

add_executable(trajectory_planner
  src/trajectory_planner_node.cpp
  src/trajectory_planner_main.cpp)
target_link_libraries(trajectory_planner ${PROJECT_NAME}_core)
ament_target_dependencies(trajectory_planner rclcpp sensor_msgs trajectory_msgs std_srvs)

install(TARGETS joint_bridge trajectory_planner
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ---- tests -----------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trapezoid test/test_trapezoid.cpp)
  target_link_libraries(test_trapezoid ${PROJECT_NAME}_core)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(arm_common)
ament_package()
