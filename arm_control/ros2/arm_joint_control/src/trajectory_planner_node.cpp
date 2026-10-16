#include "arm_joint_control/trajectory_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace arm_joint_control {

using std::placeholders::_1;
using std::placeholders::_2;

TrajectoryPlannerNode::TrajectoryPlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("trajectory_planner", options) {
  std::vector<std::string> default_names;
  for (int i = 1; i <= 6; ++i) default_names.push_back("joint" + std::to_string(i));
  joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_names);
  if (joint_names_.empty()) throw std::runtime_error("joint_names must not be empty");
  const std::size_t n = joint_names_.size();

  max_velocity_ = declare_parameter<std::vector<double>>("max_velocity", std::vector<double>(n, 1.0));
  max_acceleration_ = declare_parameter<std::vector<double>>("max_acceleration", std::vector<double>(n, 2.0));
  rate_hz_ = declare_parameter<double>("rate_hz", 100.0);
  if (max_velocity_.size() != n || max_acceleration_.size() != n) {
    throw std::runtime_error("max_velocity and max_acceleration must have one entry per joint");
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (max_velocity_[i] <= 0.0 || max_acceleration_[i] <= 0.0) {
      throw std::runtime_error("max_velocity and max_acceleration must be > 0 for " + joint_names_[i]);
    }
  }
  if (rate_hz_ <= 0.0) throw std::runtime_error("rate_hz must be > 0");

  setpoint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("joint_setpoint", 10);
  goal_sub_ = create_subscription<sensor_msgs::msg::JointState>("joint_goal", 10,
                                                                std::bind(&TrajectoryPlannerNode::onGoal, this, _1));
  state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&TrajectoryPlannerNode::onJointState, this, _1));
  stop_srv_ = create_service<std_srvs::srv::Trigger>("~/stop", std::bind(&TrajectoryPlannerNode::onStop, this, _1, _2));

  const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                             std::bind(&TrajectoryPlannerNode::onTimer, this));

  RCLCPP_INFO(get_logger(), "trajectory_planner: %zu joints, setpoints at %.0f Hz", n, rate_hz_);
}

bool TrajectoryPlannerNode::extractPositions(const sensor_msgs::msg::JointState& msg, std::vector<double>& out) const {
  const std::size_t n = joint_names_.size();
  out.assign(n, 0.0);
  if (msg.name.empty()) {
    if (msg.position.size() != n) return false;
    out = msg.position;
    return true;
  }
  if (msg.position.size() != msg.name.size()) return false;
  for (std::size_t i = 0; i < n; ++i) {
    const auto it = std::find(msg.name.begin(), msg.name.end(), joint_names_[i]);
    if (it == msg.name.end()) return false;
    out[i] = msg.position[static_cast<std::size_t>(it - msg.name.begin())];
  }
  return true;
}

void TrajectoryPlannerNode::onJointState(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  std::vector<double> q;
  if (!extractPositions(*msg, q)) return;  // some other publisher on joint_states
  std::lock_guard<std::mutex> lock(mutex_);
  measured_ = q;
  have_measured_ = true;
}

void TrajectoryPlannerNode::onGoal(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  std::vector<double> target;
  if (!extractPositions(*msg, target)) {
    RCLCPP_WARN(get_logger(), "goal ignored: needs %zu positions (optionally named)", joint_names_.size());
    return;
  }
  for (std::size_t i = 0; i < target.size(); ++i) {
    if (!std::isfinite(target[i])) {
      RCLCPP_WARN(get_logger(), "goal ignored: %s is not finite", joint_names_[i].c_str());
      return;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<double> start;
  if (have_commanded_) {
    start = commanded_;
  } else if (have_measured_) {
    start = measured_;
  } else {
    RCLCPP_WARN(get_logger(), "goal ignored: no joint_states received yet, so I don't know where the arm is");
    return;
  }
  // If a move is in progress, start the new one from the current sample so
  // there is no position jump (velocity is allowed to drop to the new profile).
  if (moving_) {
    const double t = std::chrono::duration<double>(Clock::now() - move_start_).count();
    for (std::size_t i = 0; i < start.size(); ++i) start[i] = profiles_[i].sample(t).position;
  }

  move_duration_ = planSynchronised(start, target, max_velocity_, max_acceleration_, profiles_);
  move_start_ = Clock::now();
  moving_ = move_duration_ > 0.0;
  commanded_ = moving_ ? start : target;
  have_commanded_ = true;
  RCLCPP_INFO(get_logger(), "new goal: %.2f s move", move_duration_);
}

void TrajectoryPlannerNode::onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (moving_) {
    const double t = std::chrono::duration<double>(Clock::now() - move_start_).count();
    for (std::size_t i = 0; i < profiles_.size(); ++i) commanded_[i] = profiles_[i].sample(t).position;
    moving_ = false;
    res->message = "move aborted, holding current position";
  } else {
    res->message = "no move in progress";
  }
  res->success = true;
}

void TrajectoryPlannerNode::onTimer() {
  trajectory_msgs::msg::JointTrajectoryPoint sp;
  const std::size_t n = joint_names_.size();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_commanded_) {
      if (!have_measured_) return;  // nothing to hold yet
      commanded_ = measured_;       // hold where the arm is until the first goal
      have_commanded_ = true;
    }
    if (commanded_.size() != n) commanded_.assign(n, 0.0);
    sp.positions.resize(n);
    sp.velocities.resize(n);
    sp.accelerations.resize(n);
    if (moving_) {
      const double t = std::chrono::duration<double>(Clock::now() - move_start_).count();
      for (std::size_t i = 0; i < n; ++i) {
        const auto s = profiles_[i].sample(t);
        sp.positions[i] = s.position;
        sp.velocities[i] = s.velocity;
        sp.accelerations[i] = s.acceleration;
        commanded_[i] = s.position;
      }
      if (t >= move_duration_) {
        moving_ = false;
        for (std::size_t i = 0; i < n; ++i) commanded_[i] = profiles_[i].goal();
      }
    } else {
      for (std::size_t i = 0; i < n; ++i) sp.positions[i] = commanded_[i];
    }
  }
  setpoint_pub_->publish(sp);
}

}  // namespace arm_joint_control
