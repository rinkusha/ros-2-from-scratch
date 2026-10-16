cmake_minimum_required(VERSION 3.16)

# The firmware shares its wire protocol and PID with the ROS 2 side. Override
# with -DARM_COMMON_INCLUDE_DIR=... if you build the firmware from elsewhere.
set(ARM_COMMON_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../ros2/arm_common/include"
    CACHE PATH "Path to arm_common/include (shared headers)")
if(NOT EXISTS "${ARM_COMMON_INCLUDE_DIR}/arm_common/protocol.hpp")
  message(FATAL_ERROR "arm_common headers not found at ${ARM_COMMON_INCLUDE_DIR}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_joint_controller)
