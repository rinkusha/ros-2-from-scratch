#include "control_task.hpp"

#include <algorithm>

#include "esp_check.h"
#include "esp_log.h"

namespace {
constexpr const char* kTag = "control";
constexpr UBaseType_t kInboxLength = 8;
constexpr uint32_t kTaskStackBytes = 6144;
constexpr UBaseType_t kTaskPriority = configMAX_PRIORITIES - 2;
constexpr BaseType_t kTaskCore = 1;
}  // namespace

esp_err_t ControlTask::init() {
  inbox_ = xQueueCreate(kInboxLength, sizeof(ControlMsg));
  state_out_ = xQueueCreate(1, sizeof(arm::StateMsg));
  if (inbox_ == nullptr || state_out_ == nullptr) return ESP_ERR_NO_MEM;

  ESP_RETURN_ON_ERROR(Motor::initTimer(LEDC_TIMER_0), kTag, "ledc timer");
  for (std::size_t i = 0; i < cfg::kNumJoints; ++i) {
    const auto first_channel = static_cast<ledc_channel_t>(2 * i);
    ESP_RETURN_ON_ERROR(joints_[i].init(cfg::kJoints[i], first_channel, LEDC_TIMER_0), kTag, "joint %u init",
                        static_cast<unsigned>(i));
  }
  ESP_LOGI(kTag, "%u joints initialised", static_cast<unsigned>(cfg::kNumJoints));
  return ESP_OK;
}

esp_err_t ControlTask::start() {
  const BaseType_t ok = xTaskCreatePinnedToCore(&ControlTask::taskEntry, "control", kTaskStackBytes, this,
                                                kTaskPriority, &task_, kTaskCore);
  if (ok != pdPASS) return ESP_ERR_NO_MEM;

  esp_timer_create_args_t args = {};
  args.callback = &ControlTask::timerCallback;
  args.arg = this;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "ctrl_tick";
  ESP_RETURN_ON_ERROR(esp_timer_create(&args, &timer_), kTag, "timer create");
  ESP_RETURN_ON_ERROR(esp_timer_start_periodic(timer_, cfg::kControlPeriodUs), kTag, "timer start");
  return ESP_OK;
}

void ControlTask::timerCallback(void* arg) {
  auto* self = static_cast<ControlTask*>(arg);
  if (self->task_ != nullptr) xTaskNotifyGive(self->task_);
}

void ControlTask::taskEntry(void* arg) { static_cast<ControlTask*>(arg)->run(); }

bool ControlTask::post(const ControlMsg& msg) { return xQueueSend(inbox_, &msg, 0) == pdTRUE; }

bool ControlTask::takeState(arm::StateMsg& out, TickType_t timeout) {
  return xQueueReceive(state_out_, &out, timeout) == pdTRUE;
}

void ControlTask::run() {
  int64_t last_us = esp_timer_get_time();
  uint32_t period_max_us = 0;
  uint32_t step_count = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const int64_t now_us = esp_timer_get_time();
    const uint32_t period_us = static_cast<uint32_t>(now_us - last_us);
    last_us = now_us;
    period_max_us = std::max(period_max_us, period_us);
    // Use the measured period so integrators stay correct under jitter, but
    // never let a long stall blow up the derivative term.
    const float dt = std::min(period_us, 4 * cfg::kControlPeriodUs) * 1e-6f;

    drainInbox(now_us);

    // Setpoint watchdog: the host (or the cable) went away.
    if (have_setpoint_ && (now_us - last_setpoint_us_) > static_cast<int64_t>(cfg::kSetpointTimeoutUs)) {
      for (auto& j : joints_) j.hold();
    }

    for (auto& j : joints_) j.step(dt);

    if (++step_count >= cfg::kStateDecimation) {
      step_count = 0;
      publishState(now_us, period_max_us);
      period_max_us = 0;
    }
  }
}

void ControlTask::drainInbox(int64_t now_us) {
  ControlMsg msg;
  while (xQueueReceive(inbox_, &msg, 0) == pdTRUE) {
    switch (msg.kind) {
      case ControlMsg::Kind::kSetpoint: {
        const std::size_t n = std::min<std::size_t>(msg.setpoint.joint_count, cfg::kNumJoints);
        for (std::size_t i = 0; i < n; ++i) {
          joints_[i].setSetpoint(msg.setpoint.position[i], msg.setpoint.velocity[i], msg.setpoint.effort_ff[i]);
        }
        last_setpoint_seq_ = msg.setpoint.hdr.seq;
        last_setpoint_us_ = now_us;
        have_setpoint_ = true;
        break;
      }
      case ControlMsg::Kind::kGains:
        if (msg.gains.joint < cfg::kNumJoints) joints_[msg.gains.joint].setGains(msg.gains);
        break;
      case ControlMsg::Kind::kCommand: {
        const auto cmd = static_cast<arm::Command>(msg.command.command);
        const std::size_t first = msg.command.joint == arm::kAllJoints ? 0 : msg.command.joint;
        const std::size_t last = msg.command.joint == arm::kAllJoints ? cfg::kNumJoints : first + 1;
        for (std::size_t i = first; i < last && i < cfg::kNumJoints; ++i) {
          switch (cmd) {
            case arm::Command::kEnable: joints_[i].enable(); break;
            case arm::Command::kDisable: joints_[i].disable(); break;
            case arm::Command::kZero: joints_[i].zero(); break;
            case arm::Command::kClearFault: joints_[i].clearFault(); break;
            case arm::Command::kPing: break;
          }
        }
        if (cmd == arm::Command::kEnable) {
          // A fresh enable should not immediately trip the watchdog.
          last_setpoint_us_ = now_us;
        }
        break;
      }
    }
  }
}

void ControlTask::publishState(int64_t now_us, uint32_t period_max_us) {
  arm::StateMsg st = {};
  arm::initHeader(st, arm::MsgType::kState, state_seq_++);
  st.timestamp_us = static_cast<uint32_t>(now_us);
  st.last_setpoint_seq = last_setpoint_seq_;
  const int64_t age_us = have_setpoint_ ? (now_us - last_setpoint_us_) : 0;
  st.setpoint_age_ms = static_cast<uint16_t>(std::min<int64_t>(age_us / 1000, 0xFFFF));
  st.loop_period_max_us = static_cast<uint16_t>(std::min<uint32_t>(period_max_us, 0xFFFF));
  st.joint_count = static_cast<uint8_t>(cfg::kNumJoints);
  for (std::size_t i = 0; i < cfg::kNumJoints; ++i) {
    const JointSnapshot s = joints_[i].snapshot();
    st.status[i] = static_cast<uint8_t>(s.status);
    st.fault[i] = s.fault;
    st.position[i] = s.position;
    st.velocity[i] = s.velocity;
    st.effort[i] = s.effort;
  }
  xQueueOverwrite(state_out_, &st);
}
