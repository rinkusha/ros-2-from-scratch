// TrajectoryPlannerNode: turns a joint-space goal into a stream of
// position + velocity setpoints at the control rate.
//
//   joint_goal (sensor_msgs/JointState, positions only)
//        |  trapezoidal profile per joint, time-synchronised,
//        |  limits from parameters max_velocity / max_acceleration
//        v
//   joint_setpoint (trajectory_msgs/JointTrajectoryPoint) @ 100 Hz
//
// The planner starts every move from the last *commanded* position so
// consecutive goals blend without a jump; before the first goal it holds the
// measured position read from joint_states. ~/stop (std_srvs/Trigger) aborts
// the current move and holds where the arm is.
#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "arm_joint_control/trapezoid_profile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace arm_joint_control {

class TrajectoryPlannerNode : public rclcpp::Node {
 public:
  explicit TrajectoryPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  using Clock = std::chrono::steady_clock;

  void onGoal(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void onJointState(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void onTimer();
  void onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Reorders `msg` positions into joint_names_ order. Returns false if a joint
  // is missing (or, when the message carries no names, the sizes differ).
  bool extractPositions(const sensor_msgs::msg::JointState& msg, std::vector<double>& out) const;

  std::vector<std::string> joint_names_;
  std::vector<double> max_velocity_;
  std::vector<double> max_acceleration_;
  double rate_hz_ = 100.0;

  std::mutex mutex_;
  std::vector<double> commanded_;  // last position sent (start of next move)
  bool have_commanded_ = false;
  std::vector<double> measured_;
  bool have_measured_ = false;
  std::vector<TrapezoidProfile> profiles_;
  Clock::time_point move_start_{};
  double move_duration_ = 0.0;
  bool moving_ = false;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr setpoint_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arm_joint_control
