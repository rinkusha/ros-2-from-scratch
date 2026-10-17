// One joint: encoder in, PID, motor out, plus the state machine
// (disabled / enabled / hold / fault) and the soft limits.
//
// All methods are called from the control task only; the comms task talks to
// it through the ControlTask's queue. step() is the 1 kHz loop body.
#pragma once

#include <cstdint>

#include "arm_common/filters.hpp"
#include "arm_common/pid.hpp"
#include "arm_common/protocol.hpp"
#include "board_config.hpp"
#include "encoder.hpp"
#include "esp_err.h"
#include "motor.hpp"

struct JointSnapshot {
  float position = 0.0f;  // rad
  float velocity = 0.0f;  // rad/s, filtered
  float effort = 0.0f;    // last command, [-1, 1]
  arm::JointStatus status = arm::JointStatus::kDisabled;
  uint8_t fault = arm::kFaultNone;
};

class JointController {
 public:
  esp_err_t init(const cfg::JointPins& pins, ledc_channel_t first_channel, ledc_timer_t timer);

  // Configuration from the host (GainsMsg). Returns false for invalid values.
  bool setGains(const arm::GainsMsg& g);

  // New target from the host. Non-finite values trip kFaultSetpointNaN.
  void setSetpoint(float pos, float vel, float effort_ff);

  // Commands. Return false if the command is refused in the current state.
  bool enable();
  void disable();
  bool zero();
  bool clearFault();

  // Watchdog action: hold the current measured position.
  void hold();

  // One control period. `dt` in seconds (measured, not nominal).
  void step(float dt);

  JointSnapshot snapshot() const;
  arm::JointStatus status() const { return status_; }

 private:
  void updateMeasurement(float dt);
  void trip(uint8_t fault_bit);
  float clampToLimits(float pos) const;

  Encoder encoder_;
  Motor motor_;
  arm::Pid pid_;
  arm::LowPass vel_filter_;

  float rad_per_count_ = 0.0f;
  int32_t last_count_ = 0;
  bool have_count_ = false;

  float pos_ = 0.0f;
  float vel_ = 0.0f;
  float effort_ = 0.0f;

  float sp_pos_ = 0.0f;
  float sp_vel_ = 0.0f;
  float sp_effort_ff_ = 0.0f;

  float pos_min_ = -3.14159f;
  float pos_max_ = 3.14159f;

  arm::JointStatus status_ = arm::JointStatus::kDisabled;
  uint8_t fault_ = arm::kFaultNone;
};
