#include "arm_joint_control/joint_bridge_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <sstream>
#include <stdexcept>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace arm_joint_control {

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace {

const char* statusName(uint8_t s) {
  switch (static_cast<arm::JointStatus>(s)) {
    case arm::JointStatus::kDisabled: return "disabled";
    case arm::JointStatus::kEnabled: return "enabled";
    case arm::JointStatus::kHold: return "hold (setpoint watchdog)";
    case arm::JointStatus::kFault: return "FAULT";
  }
  return "?";
}

std::string faultText(uint8_t f) {
  if (f == arm::kFaultNone) return "none";
  std::string s;
  auto add = [&](const char* name) {
    if (!s.empty()) s += "|";
    s += name;
  };
  if (f & arm::kFaultSoftLimit) add("soft_limit");
  if (f & arm::kFaultEncoder) add("encoder");
  if (f & arm::kFaultOverCurrent) add("over_current");
  if (f & arm::kFaultSetpointNaN) add("setpoint_nan");
  return s;
}

const char* ackText(arm::AckResult r) {
  switch (r) {
    case arm::AckResult::kOk: return "ok";
    case arm::AckResult::kBadJoint: return "bad joint index";
    case arm::AckResult::kBadValue: return "bad value";
    case arm::AckResult::kRefused: return "refused by firmware (fault?)";
    case arm::AckResult::kUnknown: return "unknown command";
  }
  return "?";
}

template <typename T>
std::string fmt(T v) {
  std::ostringstream os;
  os << v;
  return os.str();
}

}  // namespace

JointBridgeNode::JointBridgeNode(const rclcpp::NodeOptions& options) : rclcpp::Node("joint_bridge", options) {
  declareParameters();

  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  setpoint_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      "joint_setpoint", 10, std::bind(&JointBridgeNode::onSetpoint, this, _1));

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                     std::bind(&JointBridgeNode::onControlTimer, this));
  diag_timer_ = create_wall_timer(100ms, std::bind(&JointBridgeNode::onDiagnosticsTimer, this));

  makeTriggerService("enable", arm::Command::kEnable);
  makeTriggerService("disable", arm::Command::kDisable);
  makeTriggerService("zero", arm::Command::kZero);
  makeTriggerService("clear_fault", arm::Command::kClearFault);
  makeTriggerService("ping", arm::Command::kPing);

  param_cb_handle_ = add_on_set_parameters_callback(std::bind(&JointBridgeNode::onParametersSet, this, _1));

  running_ = true;
  reader_ = std::thread(&JointBridgeNode::readerLoop, this);

  RCLCPP_INFO(get_logger(), "joint_bridge: %zu joints, %s @ %d baud, setpoints at %.0f Hz", joints_.size(),
              port_.c_str(), baud_, control_rate_hz_);
}

JointBridgeNode::~JointBridgeNode() {
  running_ = false;
  if (reader_.joinable()) reader_.join();
  serial_.close();
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

void JointBridgeNode::declareParameters() {
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baud_ = static_cast<int>(declare_parameter<int64_t>("baud", 921600));
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 100.0);
  setpoint_stale_ms_ = static_cast<int>(declare_parameter<int64_t>("setpoint_stale_ms", 200));
  ack_timeout_ms_ = static_cast<int>(declare_parameter<int64_t>("ack_timeout_ms", 250));
  reconnect_period_ms_ = static_cast<int>(declare_parameter<int64_t>("reconnect_period_ms", 1000));

  std::vector<std::string> default_names;
  for (std::size_t i = 1; i <= arm::kMaxJoints; ++i) default_names.push_back("joint" + std::to_string(i));
  const auto names = declare_parameter<std::vector<std::string>>("joint_names", default_names);
  if (names.empty() || names.size() > arm::kMaxJoints) {
    throw std::runtime_error("joint_names must have between 1 and " + std::to_string(arm::kMaxJoints) + " entries");
  }
  if (control_rate_hz_ <= 0.0 || control_rate_hz_ > 1000.0) {
    throw std::runtime_error("control_rate_hz must be in (0, 1000]");
  }

  joints_.resize(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    joints_[i].name = names[i];
    const std::string p = names[i] + ".";
    declare_parameter<double>(p + "kp", 1.0);
    declare_parameter<double>(p + "ki", 0.0);
    declare_parameter<double>(p + "kd", 0.02);
    declare_parameter<double>(p + "kff_v", 0.0);
    declare_parameter<double>(p + "i_limit", 0.3);
    declare_parameter<double>(p + "out_limit", 0.8);
    declare_parameter<double>(p + "vel_filter_hz", 50.0);
    declare_parameter<double>(p + "pos_min", -3.14159);
    declare_parameter<double>(p + "pos_max", 3.14159);
    loadGainsFromParameters(i);
  }
}

void JointBridgeNode::loadGainsFromParameters(std::size_t joint) {
  auto& g = joints_[joint].gains;
  const std::string p = joints_[joint].name + ".";
  g.joint = static_cast<uint8_t>(joint);
  g.kp = static_cast<float>(get_parameter(p + "kp").as_double());
  g.ki = static_cast<float>(get_parameter(p + "ki").as_double());
  g.kd = static_cast<float>(get_parameter(p + "kd").as_double());
  g.kff_v = static_cast<float>(get_parameter(p + "kff_v").as_double());
  g.i_limit = static_cast<float>(get_parameter(p + "i_limit").as_double());
  g.out_limit = static_cast<float>(get_parameter(p + "out_limit").as_double());
  g.vel_filter_hz = static_cast<float>(get_parameter(p + "vel_filter_hz").as_double());
  g.pos_min = static_cast<float>(get_parameter(p + "pos_min").as_double());
  g.pos_max = static_cast<float>(get_parameter(p + "pos_max").as_double());
}

rcl_interfaces::msg::SetParametersResult JointBridgeNode::onParametersSet(
    const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::vector<bool> changed(joints_.size(), false);

  for (const auto& param : params) {
    const std::string& name = param.get_name();
    const auto dot = name.find('.');
    if (dot == std::string::npos) {
      // Link parameters are read once at start-up.
      if (name == "port" || name == "baud" || name == "joint_names" || name == "control_rate_hz") {
        result.successful = false;
        result.reason = name + " cannot be changed at runtime; restart the node";
        return result;
      }
      continue;
    }
    const std::string joint_name = name.substr(0, dot);
    const std::string field = name.substr(dot + 1);
    const auto it = std::find_if(joints_.begin(), joints_.end(),
                                 [&](const JointConfig& j) { return j.name == joint_name; });
    if (it == joints_.end()) continue;
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.successful = false;
      result.reason = name + " must be a double";
      return result;
    }
    const float v = static_cast<float>(param.as_double());
    auto& g = it->gains;
    if (field == "kp") g.kp = v;
    else if (field == "ki") g.ki = v;
    else if (field == "kd") g.kd = v;
    else if (field == "kff_v") g.kff_v = v;
    else if (field == "i_limit") g.i_limit = v;
    else if (field == "out_limit") {
      if (v <= 0.0f || v > 1.0f) {
        result.successful = false;
        result.reason = name + " must be in (0, 1]";
        return result;
      }
      g.out_limit = v;
    } else if (field == "vel_filter_hz") g.vel_filter_hz = v;
    else if (field == "pos_min") g.pos_min = v;
    else if (field == "pos_max") g.pos_max = v;
    else continue;
    changed[static_cast<std::size_t>(it - joints_.begin())] = true;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto& g = joints_[i].gains;
    if (changed[i] && g.pos_min >= g.pos_max) {
      result.successful = false;
      result.reason = joints_[i].name + ": pos_min must be < pos_max";
      return result;
    }
  }
  if (connected_) {
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      if (changed[i]) {
        if (sendGains(i)) {
          RCLCPP_INFO(get_logger(), "%s: gains updated (kp=%.3f ki=%.3f kd=%.3f kff_v=%.3f)", joints_[i].name.c_str(),
                      joints_[i].gains.kp, joints_[i].gains.ki, joints_[i].gains.kd, joints_[i].gains.kff_v);
        } else {
          RCLCPP_WARN(get_logger(), "%s: gains stored but firmware did not acknowledge", joints_[i].name.c_str());
        }
      }
    }
  }
  return result;
}

// ---------------------------------------------------------------------------
// Link
// ---------------------------------------------------------------------------

bool JointBridgeNode::connect() {
  try {
    serial_.open(port_, baud_);
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cannot open %s: %s (retrying every %d ms)", port_.c_str(),
                         e.what(), reconnect_period_ms_);
    return false;
  }
  serial_.flushInput();
  RCLCPP_INFO(get_logger(), "connected to %s", port_.c_str());
  return true;
}

void JointBridgeNode::readerLoop() {
  uint8_t buf[512];
  arm::FrameParser parser;
  while (running_) {
    if (!serial_.isOpen()) {
      if (!connect()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_period_ms_));
        continue;
      }
      connected_ = true;
      // Firmware may have just rebooted: ask the control timer to push the
      // configuration. (Not done here: the acks arrive on this thread.)
      gains_push_pending_ = true;
    }
    const int n = serial_.read(buf, sizeof(buf), 50);
    if (n < 0) {
      RCLCPP_ERROR(get_logger(), "serial read error on %s, reconnecting", port_.c_str());
      serial_.close();
      connected_ = false;
      continue;
    }
    for (int i = 0; i < n; ++i) {
      if (parser.feed(buf[i])) dispatchFrame(parser);
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      frames_ok_ = parser.framesOk();
      frames_bad_ = parser.framesBad();
    }
  }
}

void JointBridgeNode::dispatchFrame(const arm::FrameParser& parser) {
  if (parser.is<arm::StateMsg>(arm::MsgType::kState)) {
    handleState(parser.as<arm::StateMsg>());
  } else if (parser.is<arm::AckMsg>(arm::MsgType::kAck)) {
    handleAck(parser.as<arm::AckMsg>());
  } else if (parser.isType(arm::MsgType::kLog)) {
    handleLog(parser.payload(), parser.size());
  } else {
    RCLCPP_DEBUG(get_logger(), "unknown frame: %zu bytes, type 0x%02x", parser.size(),
                 parser.size() > 1 ? parser.payload()[1] : 0);
  }
}

bool JointBridgeNode::sendRaw(const void* payload, std::size_t len) {
  uint8_t frame[arm::kMaxFrameSize];
  const std::size_t n = arm::encodeFrame(payload, len, frame, sizeof(frame));
  if (n == 0) return false;
  return serial_.write(frame, n);
}

template <typename T>
bool JointBridgeNode::send(T& msg, arm::MsgType type) {
  arm::initHeader(msg, type, tx_seq_.fetch_add(1));
  return sendRaw(&msg, sizeof(T));
}

bool JointBridgeNode::sendGains(std::size_t joint) {
  std::lock_guard<std::mutex> txn(txn_mutex_);
  arm::GainsMsg g = joints_[joint].gains;
  g.joint = static_cast<uint8_t>(joint);
  const uint16_t seq = tx_seq_.fetch_add(1);
  arm::initHeader(g, arm::MsgType::kGains, seq);
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    ack_pending_ = true;
    ack_received_ = false;
    ack_wait_seq_ = seq;
  }
  if (!sendRaw(&g, sizeof(g))) {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    ack_pending_ = false;
    return false;
  }
  std::unique_lock<std::mutex> lock(ack_mutex_);
  const bool ok = ack_cv_.wait_for(lock, std::chrono::milliseconds(ack_timeout_ms_), [&] { return ack_received_; }) &&
                  ack_result_ == arm::AckResult::kOk;
  ack_pending_ = false;
  return ok;
}

bool JointBridgeNode::sendAllGains() {
  bool all_ok = true;
  for (std::size_t i = 0; i < joints_.size(); ++i) all_ok &= sendGains(i);
  return all_ok;
}

bool JointBridgeNode::sendCommand(arm::Command cmd, uint8_t joint, std::string& message) {
  if (!connected_) {
    message = "not connected to " + port_;
    return false;
  }
  std::lock_guard<std::mutex> txn(txn_mutex_);
  arm::CommandMsg m{};
  m.command = static_cast<uint8_t>(cmd);
  m.joint = joint;
  const uint16_t seq = tx_seq_.fetch_add(1);
  arm::initHeader(m, arm::MsgType::kCommand, seq);
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    ack_pending_ = true;
    ack_received_ = false;
    ack_wait_seq_ = seq;
  }
  if (!sendRaw(&m, sizeof(m))) {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    ack_pending_ = false;
    message = "serial write failed";
    return false;
  }
  std::unique_lock<std::mutex> lock(ack_mutex_);
  const bool got = ack_cv_.wait_for(lock, std::chrono::milliseconds(ack_timeout_ms_), [&] { return ack_received_; });
  ack_pending_ = false;
  if (!got) {
    message = "no acknowledgement from firmware within " + std::to_string(ack_timeout_ms_) + " ms";
    return false;
  }
  message = ackText(ack_result_);
  return ack_result_ == arm::AckResult::kOk;
}

// ---------------------------------------------------------------------------
// ROS callbacks
// ---------------------------------------------------------------------------

void JointBridgeNode::makeTriggerService(const std::string& name, arm::Command cmd) {
  auto handler = [this, cmd, name](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
    res->success = sendCommand(cmd, arm::kAllJoints, res->message);
    if (res->success) {
      RCLCPP_INFO(get_logger(), "%s: ok", name.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s: %s", name.c_str(), res->message.c_str());
    }
  };
  services_.push_back(create_service<Trigger>("~/" + name, handler, rmw_qos_profile_services_default, service_group_));
}

void JointBridgeNode::onSetpoint(trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr msg) {
  if (msg->positions.size() != joints_.size()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "setpoint has %zu positions, expected %zu; ignored",
                         msg->positions.size(), joints_.size());
    return;
  }
  std::lock_guard<std::mutex> lock(setpoint_mutex_);
  latest_setpoint_ = *msg;
  latest_setpoint_time_ = Clock::now();
  have_setpoint_ = true;
}

void JointBridgeNode::onControlTimer() {
  if (!connected_) return;
  if (gains_push_pending_.exchange(false)) {
    if (sendAllGains()) {
      RCLCPP_INFO(get_logger(), "gains pushed to firmware for %zu joints", joints_.size());
    } else {
      RCLCPP_WARN(get_logger(), "firmware did not acknowledge all gains; will retry on next reconnect");
    }
  }
  arm::SetpointMsg sp{};
  {
    std::lock_guard<std::mutex> lock(setpoint_mutex_);
    if (!have_setpoint_) return;
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - latest_setpoint_time_);
    if (age.count() > setpoint_stale_ms_) {
      // Stop forwarding: the firmware watchdog will put the joints in HOLD.
      setpoints_dropped_.fetch_add(1);
      return;
    }
    sp.joint_count = static_cast<uint8_t>(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      sp.position[i] = static_cast<float>(latest_setpoint_.positions[i]);
      sp.velocity[i] = i < latest_setpoint_.velocities.size() ? static_cast<float>(latest_setpoint_.velocities[i]) : 0.0f;
      sp.effort_ff[i] = i < latest_setpoint_.effort.size() ? static_cast<float>(latest_setpoint_.effort[i]) : 0.0f;
    }
  }
  if (send(sp, arm::MsgType::kSetpoint)) {
    setpoints_sent_.fetch_add(1);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "setpoint write failed");
  }
}

void JointBridgeNode::handleState(const arm::StateMsg& st) {
  const std::size_t n = std::min<std::size_t>(st.joint_count, joints_.size());

  sensor_msgs::msg::JointState js;
  js.header.stamp = now();
  js.name.reserve(n);
  js.position.reserve(n);
  js.velocity.reserve(n);
  js.effort.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    js.name.push_back(joints_[i].name);
    js.position.push_back(st.position[i]);
    js.velocity.push_back(st.velocity[i]);
    js.effort.push_back(st.effort[i]);
  }
  joint_state_pub_->publish(js);

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (have_state_) {
    for (std::size_t i = 0; i < n; ++i) {
      if (st.status[i] != last_state_.status[i] || st.fault[i] != last_state_.fault[i]) {
        const bool is_fault = st.status[i] == static_cast<uint8_t>(arm::JointStatus::kFault);
        if (is_fault) {
          RCLCPP_ERROR(get_logger(), "%s: %s -> %s [%s]", joints_[i].name.c_str(), statusName(last_state_.status[i]),
                       statusName(st.status[i]), faultText(st.fault[i]).c_str());
        } else {
          RCLCPP_INFO(get_logger(), "%s: %s -> %s", joints_[i].name.c_str(), statusName(last_state_.status[i]),
                      statusName(st.status[i]));
        }
      }
    }
  } else if (st.joint_count != joints_.size()) {
    RCLCPP_WARN(get_logger(), "firmware reports %u joints, this node is configured for %zu",
                static_cast<unsigned>(st.joint_count), joints_.size());
  }
  last_state_ = st;
  have_state_ = true;
  last_state_time_ = Clock::now();
}

void JointBridgeNode::handleAck(const arm::AckMsg& ack) {
  std::lock_guard<std::mutex> lock(ack_mutex_);
  if (ack_pending_ && ack.acked_seq == ack_wait_seq_) {
    ack_received_ = true;
    ack_result_ = static_cast<arm::AckResult>(ack.result);
    ack_cv_.notify_all();
  }
}

void JointBridgeNode::handleLog(const uint8_t* payload, std::size_t len) {
  if (len <= sizeof(arm::Header) + 1) return;
  const uint8_t level = payload[sizeof(arm::Header)];
  const std::string text(reinterpret_cast<const char*>(payload) + sizeof(arm::Header) + 1,
                         len - sizeof(arm::Header) - 1);
  switch (level) {
    case 0: RCLCPP_ERROR(get_logger(), "[fw] %s", text.c_str()); break;
    case 1: RCLCPP_WARN(get_logger(), "[fw] %s", text.c_str()); break;
    default: RCLCPP_INFO(get_logger(), "[fw] %s", text.c_str()); break;
  }
}

void JointBridgeNode::onDiagnosticsTimer() {
  using diagnostic_msgs::msg::DiagnosticStatus;
  using diagnostic_msgs::msg::KeyValue;
  auto kv = [](const std::string& k, const std::string& v) {
    KeyValue e;
    e.key = k;
    e.value = v;
    return e;
  };

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();

  arm::StateMsg st{};
  bool have = false;
  double state_age_ms = 0.0;
  uint32_t ok = 0, bad = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    st = last_state_;
    have = have_state_;
    ok = frames_ok_;
    bad = frames_bad_;
    if (have) {
      state_age_ms = std::chrono::duration<double, std::milli>(Clock::now() - last_state_time_).count();
    }
  }

  DiagnosticStatus link;
  link.name = std::string(get_name()) + ": link";
  link.hardware_id = port_;
  if (!connected_) {
    link.level = DiagnosticStatus::ERROR;
    link.message = "serial port not open";
  } else if (!have || state_age_ms > 500.0) {
    link.level = DiagnosticStatus::ERROR;
    link.message = "no state from firmware";
  } else if (st.loop_period_max_us > 1500) {
    link.level = DiagnosticStatus::WARN;
    link.message = "firmware loop jitter";
  } else {
    link.level = DiagnosticStatus::OK;
    link.message = "ok";
  }
  link.values.push_back(kv("frames_ok", fmt(ok)));
  link.values.push_back(kv("frames_bad", fmt(bad)));
  link.values.push_back(kv("setpoints_sent", fmt(setpoints_sent_.load())));
  link.values.push_back(kv("setpoints_dropped_stale", fmt(setpoints_dropped_.load())));
  link.values.push_back(kv("state_age_ms", fmt(state_age_ms)));
  link.values.push_back(kv("fw_setpoint_age_ms", fmt(st.setpoint_age_ms)));
  link.values.push_back(kv("fw_loop_period_max_us", fmt(st.loop_period_max_us)));
  arr.status.push_back(link);

  const std::size_t n = have ? std::min<std::size_t>(st.joint_count, joints_.size()) : 0;
  for (std::size_t i = 0; i < n; ++i) {
    DiagnosticStatus js;
    js.name = std::string(get_name()) + ": " + joints_[i].name;
    js.hardware_id = port_;
    switch (static_cast<arm::JointStatus>(st.status[i])) {
      case arm::JointStatus::kEnabled: js.level = DiagnosticStatus::OK; break;
      case arm::JointStatus::kFault: js.level = DiagnosticStatus::ERROR; break;
      default: js.level = DiagnosticStatus::WARN; break;
    }
    js.message = statusName(st.status[i]);
    js.values.push_back(kv("fault", faultText(st.fault[i])));
    js.values.push_back(kv("position", fmt(st.position[i])));
    js.values.push_back(kv("velocity", fmt(st.velocity[i])));
    js.values.push_back(kv("effort", fmt(st.effort[i])));
    arr.status.push_back(js);
  }
  diag_pub_->publish(arr);
}

}  // namespace arm_joint_control
