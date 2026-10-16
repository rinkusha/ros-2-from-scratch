#include <cstdio>
#include <exception>

#include "arm_joint_control/trajectory_planner_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<arm_joint_control::TrajectoryPlannerNode>());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "trajectory_planner: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
