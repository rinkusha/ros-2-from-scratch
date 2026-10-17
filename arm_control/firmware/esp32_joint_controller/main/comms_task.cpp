#include "comms_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esp_log.h"

namespace {
constexpr const char* kTag = "comms";
constexpr uint32_t kTaskStackBytes = 6144;
constexpr UBaseType_t kTaskPriority = 10;
constexpr BaseType_t kTaskCore = 0;
constexpr TickType_t kReadTimeout = 1;  // ticks (1 ms at CONFIG_FREERTOS_HZ=1000)
}  // namespace

CommsTask* CommsTask::log_sink_ = nullptr;

esp_err_t CommsTask::start() {
  const BaseType_t ok =
      xTaskCreatePinnedToCore(&CommsTask::taskEntry, "comms", kTaskStackBytes, this, kTaskPriority, &task_, kTaskCore);
  return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void CommsTask::installLogHook() {
  log_sink_ = this;
  esp_log_set_vprintf(&CommsTask::logVprintf);
}

int CommsTask::logVprintf(const char* fmt, va_list args) {
  if (log_sink_ == nullptr || !log_sink_->link_.ready()) return 0;
  char line[96];
  int n = vsnprintf(line, sizeof(line), fmt, args);
  if (n <= 0) return 0;
  if (n > static_cast<int>(sizeof(line) - 1)) n = sizeof(line) - 1;
  // Strip the trailing newline and ANSI colour codes ESP_LOG adds.
  while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) --n;
  const char* start = line;
  if (start[0] == '\033') {
    const char* m = std::strchr(start, 'm');
    if (m != nullptr) start = m + 1;
  }
  int len = static_cast<int>(n - (start - line));
  if (len > 0 && start[len - 1] == 'm') {
    const char* esc = std::strrchr(start, '\033');
    if (esc != nullptr) len = static_cast<int>(esc - start);
  }
  if (len <= 0) return n;

  arm::LogMsg msg = {};
  arm::initHeader(msg, arm::MsgType::kLog, 0);
  msg.level = (start[0] == 'E') ? 0 : (start[0] == 'W') ? 1 : 2;
  const int copy = std::min<int>(len, static_cast<int>(sizeof(msg.text)));
  std::memcpy(msg.text, start, copy);
  log_sink_->link_.sendFrame(&msg, sizeof(arm::Header) + 1 + copy);
  return n;
}

void CommsTask::taskEntry(void* arg) { static_cast<CommsTask*>(arg)->run(); }

void CommsTask::ack(arm::MsgType type, uint16_t seq, arm::AckResult result) {
  arm::AckMsg a = {};
  arm::initHeader(a, arm::MsgType::kAck, tx_seq_++);
  a.acked_type = static_cast<uint8_t>(type);
  a.acked_seq = seq;
  a.result = static_cast<uint8_t>(result);
  link_.send(a);
}

void CommsTask::run() {
  uint8_t buf[256];
  arm::StateMsg state;
  for (;;) {
    const int n = link_.read(buf, sizeof(buf), kReadTimeout);
    for (int i = 0; i < n; ++i) {
      if (parser_.feed(buf[i])) handleFrame();
    }
    // Forward the newest state (the control task overwrites, so at most one).
    while (control_.takeState(state, 0)) {
      state.hdr.seq = tx_seq_++;
      link_.send(state);
    }
  }
}

void CommsTask::handleFrame() {
  using arm::MsgType;

  if (parser_.is<arm::SetpointMsg>(MsgType::kSetpoint)) {
    ControlMsg m;
    m.kind = ControlMsg::Kind::kSetpoint;
    m.setpoint = parser_.as<arm::SetpointMsg>();
    if (m.setpoint.joint_count > arm::kMaxJoints) return;  // malformed, no ack for setpoints
    control_.post(m);  // a dropped setpoint is harmless: the next one is 10 ms away
    return;
  }

  if (parser_.is<arm::GainsMsg>(MsgType::kGains)) {
    ControlMsg m;
    m.kind = ControlMsg::Kind::kGains;
    m.gains = parser_.as<arm::GainsMsg>();
    const auto& g = m.gains;
    arm::AckResult result = arm::AckResult::kOk;
    if (g.joint >= ControlTask::numJoints()) {
      result = arm::AckResult::kBadJoint;
    } else if (!std::isfinite(g.kp) || !std::isfinite(g.ki) || !std::isfinite(g.kd) || !std::isfinite(g.kff_v) ||
               !std::isfinite(g.i_limit) || !std::isfinite(g.out_limit) || !std::isfinite(g.vel_filter_hz) ||
               !std::isfinite(g.pos_min) || !std::isfinite(g.pos_max) || g.out_limit <= 0.0f || g.out_limit > 1.0f ||
               g.pos_min >= g.pos_max || g.i_limit < 0.0f || g.vel_filter_hz < 0.0f) {
      result = arm::AckResult::kBadValue;
    } else if (!control_.post(m)) {
      result = arm::AckResult::kRefused;
    }
    ack(MsgType::kGains, g.hdr.seq, result);
    return;
  }

  if (parser_.is<arm::CommandMsg>(MsgType::kCommand)) {
    ControlMsg m;
    m.kind = ControlMsg::Kind::kCommand;
    m.command = parser_.as<arm::CommandMsg>();
    const auto& c = m.command;
    const auto cmd = static_cast<arm::Command>(c.command);
    arm::AckResult result = arm::AckResult::kOk;

    if (c.joint != arm::kAllJoints && c.joint >= ControlTask::numJoints()) {
      result = arm::AckResult::kBadJoint;
    } else {
      const std::size_t first = c.joint == arm::kAllJoints ? 0 : c.joint;
      const std::size_t last = c.joint == arm::kAllJoints ? ControlTask::numJoints() : first + 1;
      switch (cmd) {
        case arm::Command::kPing:
          ack(MsgType::kCommand, c.hdr.seq, arm::AckResult::kOk);
          return;  // nothing to post
        case arm::Command::kEnable:
          for (std::size_t i = first; i < last; ++i) {
            if (control_.jointStatus(i) == arm::JointStatus::kFault) result = arm::AckResult::kRefused;
          }
          break;
        case arm::Command::kZero:
          for (std::size_t i = first; i < last; ++i) {
            const auto s = control_.jointStatus(i);
            if (s == arm::JointStatus::kEnabled || s == arm::JointStatus::kHold) result = arm::AckResult::kRefused;
          }
          break;
        case arm::Command::kDisable:
        case arm::Command::kClearFault:
          break;
        default:
          result = arm::AckResult::kUnknown;
          break;
      }
      if (result == arm::AckResult::kOk && !control_.post(m)) result = arm::AckResult::kRefused;
    }
    ack(MsgType::kCommand, c.hdr.seq, result);
    return;
  }

  ESP_LOGW(kTag, "unknown frame type 0x%02x (%u bytes)", parser_.size() > 1 ? parser_.payload()[1] : 0,
           static_cast<unsigned>(parser_.size()));
}
