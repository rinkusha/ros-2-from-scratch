// Position PID with velocity feed-forward, written for a fixed-rate loop on
// a microcontroller but kept header-only and dependency-free so the host can
// unit-test it and simulate it.
//
//   u = kp*e + ki*∫e dt  - kd*v_meas + kff_v*v_sp + effort_ff
//
// Design choices (the ones that matter on real hardware):
//   * Derivative on measurement, not on error. A step in the setpoint then
//     does not spike the output ("derivative kick").
//   * Conditional integration for anti-windup: the integral stops growing
//     while the output is saturated in the same direction as the error, and
//     is additionally clamped to ±i_limit.
//   * Output clamped to ±out_limit so a mis-tuned gain cannot exceed the
//     motor driver's duty range.
#pragma once

#include <cmath>

namespace arm {

struct PidGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float kff_v = 0.0f;
  float i_limit = 0.0f;    // 0 disables the clamp
  float out_limit = 1.0f;  // normalised motor command
};

class Pid {
 public:
  void setGains(const PidGains& g) { g_ = g; }
  const PidGains& gains() const { return g_; }

  void reset() {
    integral_ = 0.0f;
    last_output_ = 0.0f;
    saturated_ = false;
  }

  // Returns the clamped command. `dt` in seconds.
  float update(float sp_pos, float sp_vel, float meas_pos, float meas_vel, float dt, float effort_ff = 0.0f) {
    const float e = sp_pos - meas_pos;

    // Conditional integration: freeze when saturated and the error would push
    // further into saturation.
    const bool pushing_into_sat = saturated_ && ((e > 0.0f && last_output_ > 0.0f) || (e < 0.0f && last_output_ < 0.0f));
    if (!pushing_into_sat && dt > 0.0f) {
      integral_ += g_.ki * e * dt;
      if (g_.i_limit > 0.0f) {
        if (integral_ > g_.i_limit) integral_ = g_.i_limit;
        if (integral_ < -g_.i_limit) integral_ = -g_.i_limit;
      }
    }

    float u = g_.kp * e + integral_ - g_.kd * meas_vel + g_.kff_v * sp_vel + effort_ff;

    const float lim = g_.out_limit > 0.0f ? g_.out_limit : 1.0f;
    if (u > lim) {
      u = lim;
      saturated_ = true;
    } else if (u < -lim) {
      u = -lim;
      saturated_ = true;
    } else {
      saturated_ = false;
    }
    last_output_ = u;
    return u;
  }

  float integral() const { return integral_; }
  bool saturated() const { return saturated_; }

 private:
  PidGains g_{};
  float integral_ = 0.0f;
  float last_output_ = 0.0f;
  bool saturated_ = false;
};

}  // namespace arm
