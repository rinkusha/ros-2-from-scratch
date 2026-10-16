#include <cstdio>
#include <exception>

#include "arm_joint_control/joint_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<arm_joint_control::JointBridgeNode>();
    // Multi-threaded so a service waiting for a firmware ack never delays the
    // 100 Hz setpoint timer.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "joint_bridge: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
