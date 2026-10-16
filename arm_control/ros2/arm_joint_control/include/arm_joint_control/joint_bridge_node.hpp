// JointBridgeNode: the ROS 2 side of the ESP32 serial link.
//
//   joint_setpoint (trajectory_msgs/JointTrajectoryPoint) --+
//                                                           v
//   [100 Hz timer] -> SetpointMsg -> UART -> ESP32 (1 kHz PID per joint)
//   [reader thread] <- StateMsg  <- UART <-   "
//        |-> joint_states (sensor_msgs/JointState)
//        |-> diagnostics (diagnostic_msgs/DiagnosticArray, 10 Hz)
//
// Services (std_srvs/Trigger): ~/enable ~/disable ~/zero ~/clear_fault ~/ping
// Per-joint gains are ROS parameters (<joint>.kp, .ki, ...) and are pushed to
// the firmware on start-up, on reconnect, and whenever a parameter changes.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "arm_common/framing.hpp"
#include "arm_common/protocol.hpp"
#include "arm_joint_control/serial_link.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace arm_joint_control {

class JointBridgeNode : public rclcpp::Node {
 public:
  explicit JointBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~JointBridgeNode() override;

 private:
  using Clock = std::chrono::steady_clock;
  using Trigger = std_srvs::srv::Trigger;

  struct JointConfig {
    std::string name;
    arm::GainsMsg gains{};
  };

  // --- parameters -----------------------------------------------------------
  void declareParameters();
  void loadGainsFromParameters(std::size_t joint);
  rcl_interfaces::msg::SetParametersResult onParametersSet(const std::vector<rclcpp::Parameter>& params);

  // --- link -----------------------------------------------------------------
  bool connect();
  void readerLoop();
  void dispatchFrame(const arm::FrameParser& parser);
  bool sendRaw(const void* payload, std::size_t len);
  template <typename T>
  bool send(T& msg, arm::MsgType type);
  bool sendGains(std::size_t joint);
  bool sendAllGains();
  // Sends a command and waits for its Ack. Returns true on AckResult::kOk.
  bool sendCommand(arm::Command cmd, uint8_t joint, std::string& message);

  // --- ROS callbacks --------------------------------------------------------
  void onSetpoint(trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr msg);
  void onControlTimer();
  void onDiagnosticsTimer();
  void handleState(const arm::StateMsg& st);
  void handleAck(const arm::AckMsg& ack);
  void handleLog(const uint8_t* payload, std::size_t len);
  void makeTriggerService(const std::string& name, arm::Command cmd);

  // --- configuration --------------------------------------------------------
  std::string port_;
  int baud_ = 921600;
  double control_rate_hz_ = 100.0;
  int setpoint_stale_ms_ = 200;
  int ack_timeout_ms_ = 250;
  int reconnect_period_ms_ = 1000;
  std::vector<JointConfig> joints_;

  // --- link state -----------------------------------------------------------
  SerialLink serial_;
  std::thread reader_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<uint16_t> tx_seq_{0};

  // --- setpoint buffer ------------------------------------------------------
  std::mutex setpoint_mutex_;
  trajectory_msgs::msg::JointTrajectoryPoint latest_setpoint_;
  Clock::time_point latest_setpoint_time_{};
  bool have_setpoint_ = false;
  std::atomic<uint64_t> setpoints_sent_{0};
  std::atomic<uint64_t> setpoints_dropped_{0};

  // --- ack rendezvous -------------------------------------------------------
  // One acknowledged transaction (gains or command) at a time. Never used from
  // the reader thread, which is the one that delivers the ack.
  std::mutex txn_mutex_;
  std::atomic<bool> gains_push_pending_{false};
  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;
  uint16_t ack_wait_seq_ = 0;
  bool ack_pending_ = false;
  bool ack_received_ = false;
  arm::AckResult ack_result_ = arm::AckResult::kUnknown;

  // --- last state for diagnostics ------------------------------------------
  std::mutex state_mutex_;
  arm::StateMsg last_state_{};
  bool have_state_ = false;
  Clock::time_point last_state_time_{};
  uint32_t frames_ok_ = 0;
  uint32_t frames_bad_ = 0;

  // --- ROS interfaces -------------------------------------------------------
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace arm_joint_control
