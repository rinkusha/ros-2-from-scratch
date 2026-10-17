// The 1 kHz control loop. Runs as a FreeRTOS task pinned to core 1, woken by
// a periodic esp_timer. Owns every JointController. Receives host messages
// through a queue (filled by the comms task) and publishes a StateMsg every
// kStateDecimation steps through a length-1 overwrite queue.
#pragma once

#include <cstddef>
#include <cstdint>

#include "arm_common/protocol.hpp"
#include "board_config.hpp"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "joint_controller.hpp"

// Anything the comms task hands to the control loop. Validation of ranges and
// joint indices happens in the comms task before enqueueing.
struct ControlMsg {
  enum class Kind : uint8_t { kSetpoint, kGains, kCommand };
  Kind kind;
  union {
    arm::SetpointMsg setpoint;
    arm::GainsMsg gains;
    arm::CommandMsg command;
  };
};

class ControlTask {
 public:
  esp_err_t init();
  esp_err_t start();

  // Comms -> control. Non-blocking; returns false if the queue is full.
  bool post(const ControlMsg& msg);

  // Control -> comms. Returns true and fills `out` when a new state is ready.
  bool takeState(arm::StateMsg& out, TickType_t timeout);

  // Racy-but-harmless read for the comms task's ack decisions.
  arm::JointStatus jointStatus(std::size_t i) const { return joints_[i].status(); }

  static constexpr std::size_t numJoints() { return cfg::kNumJoints; }

 private:
  static void timerCallback(void* arg);
  static void taskEntry(void* arg);
  void run();
  void drainInbox(int64_t now_us);
  void publishState(int64_t now_us, uint32_t period_max_us);

  JointController joints_[cfg::kNumJoints];
  QueueHandle_t inbox_ = nullptr;
  QueueHandle_t state_out_ = nullptr;
  TaskHandle_t task_ = nullptr;
  esp_timer_handle_t timer_ = nullptr;

  uint16_t last_setpoint_seq_ = 0;
  int64_t last_setpoint_us_ = 0;
  bool have_setpoint_ = false;
  uint16_t state_seq_ = 0;
};
