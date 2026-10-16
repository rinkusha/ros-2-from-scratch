#include "joint_controller.hpp"

#include <cmath>

#include "esp_check.h"

namespace {
constexpr const char* kTag = "joint";
constexpr float kTwoPi = 6.28318530718f;

bool finite(float v) { return std::isfinite(v); }
}  // namespace

esp_err_t JointController::init(const cfg::JointPins& pins, ledc_channel_t first_channel, ledc_timer_t timer) {
  ESP_RETURN_ON_ERROR(encoder_.init(pins.enc_a, pins.enc_b, pins.invert_encoder), kTag, "encoder");
  ESP_RETURN_ON_ERROR(motor_.init(pins, first_channel, timer), kTag, "motor");
  rad_per_count_ = pins.counts_per_rev > 0.0f ? kTwoPi / pins.counts_per_rev : 0.0f;

  arm::PidGains g;
  g.kp = cfg::kDefaultKp;
  g.kd = cfg::kDefaultKd;
  g.out_limit = cfg::kDefaultOutLimit;
  pid_.setGains(g);
  vel_filter_.setCutoff(cfg::kDefaultVelFilterHz);

  last_count_ = encoder_.count();
  have_count_ = true;
  pos_ = static_cast<float>(last_count_) * rad_per_count_;
  vel_filter_.reset(0.0f);
  status_ = arm::JointStatus::kDisabled;
  motor_.coast();
  return ESP_OK;
}

bool JointController::setGains(const arm::GainsMsg& g) {
  if (!finite(g.kp) || !finite(g.ki) || !finite(g.kd) || !finite(g.kff_v) || !finite(g.i_limit) ||
      !finite(g.out_limit) || !finite(g.vel_filter_hz) || !finite(g.pos_min) || !finite(g.pos_max)) {
    return false;
  }
  if (g.out_limit <= 0.0f || g.out_limit > 1.0f || g.pos_min >= g.pos_max || g.i_limit < 0.0f ||
      g.vel_filter_hz < 0.0f) {
    return false;
  }
  arm::PidGains pg;
  pg.kp = g.kp;
  pg.ki = g.ki;
  pg.kd = g.kd;
  pg.kff_v = g.kff_v;
  pg.i_limit = g.i_limit;
  pg.out_limit = g.out_limit;
  pid_.setGains(pg);
  vel_filter_.setCutoff(g.vel_filter_hz);
  pos_min_ = g.pos_min;
  pos_max_ = g.pos_max;
  return true;
}

float JointController::clampToLimits(float pos) const {
  if (pos < pos_min_) return pos_min_;
  if (pos > pos_max_) return pos_max_;
  return pos;
}

void JointController::setSetpoint(float pos, float vel, float effort_ff) {
  if (!finite(pos) || !finite(vel) || !finite(effort_ff)) {
    trip(arm::kFaultSetpointNaN);
    return;
  }
  sp_pos_ = clampToLimits(pos);
  sp_vel_ = vel;
  sp_effort_ff_ = effort_ff;
  if (status_ == arm::JointStatus::kHold) status_ = arm::JointStatus::kEnabled;  // watchdog recovered
}

bool JointController::enable() {
  if (status_ == arm::JointStatus::kFault) return false;
  if (status_ == arm::JointStatus::kDisabled) {
    // Bumpless start: target where we are, integrator empty.
    sp_pos_ = clampToLimits(pos_);
    sp_vel_ = 0.0f;
    sp_effort_ff_ = 0.0f;
    pid_.reset();
    status_ = arm::JointStatus::kHold;  // promoted to kEnabled by the first setpoint
  }
  return true;
}

void JointController::disable() {
  if (status_ == arm::JointStatus::kFault) return;  // fault keeps its brake
  status_ = arm::JointStatus::kDisabled;
  effort_ = 0.0f;
  motor_.coast();
}

bool JointController::zero() {
  if (status_ == arm::JointStatus::kEnabled || status_ == arm::JointStatus::kHold) return false;  // disable first
  if (encoder_.zero() != ESP_OK) return false;
  last_count_ = 0;
  pos_ = 0.0f;
  vel_ = 0.0f;
  vel_filter_.reset(0.0f);
  sp_pos_ = 0.0f;
  return true;
}

bool JointController::clearFault() {
  if (status_ != arm::JointStatus::kFault) return true;
  fault_ = arm::kFaultNone;
  status_ = arm::JointStatus::kDisabled;
  motor_.coast();
  effort_ = 0.0f;
  return true;
}

void JointController::hold() {
  if (status_ != arm::JointStatus::kEnabled) return;
  sp_pos_ = clampToLimits(pos_);
  sp_vel_ = 0.0f;
  sp_effort_ff_ = 0.0f;
  status_ = arm::JointStatus::kHold;
}

void JointController::trip(uint8_t fault_bit) {
  fault_ |= fault_bit;
  status_ = arm::JointStatus::kFault;
  effort_ = 0.0f;
  motor_.brake();
}

void JointController::updateMeasurement(float dt) {
  const int32_t c = encoder_.count();
  const int32_t delta = c - last_count_;
  last_count_ = c;
  pos_ = static_cast<float>(c) * rad_per_count_;
  const float raw_vel = dt > 0.0f ? static_cast<float>(delta) * rad_per_count_ / dt : 0.0f;
  vel_ = vel_filter_.update(raw_vel, dt);
}

void JointController::step(float dt) {
  updateMeasurement(dt);

  switch (status_) {
    case arm::JointStatus::kEnabled:
    case arm::JointStatus::kHold: {
      if (pos_ < pos_min_ - cfg::kSoftLimitMarginRad || pos_ > pos_max_ + cfg::kSoftLimitMarginRad) {
        trip(arm::kFaultSoftLimit);
        return;
      }
      effort_ = pid_.update(sp_pos_, sp_vel_, pos_, vel_, dt, sp_effort_ff_);
      motor_.set(effort_);
      break;
    }
    case arm::JointStatus::kDisabled:
    case arm::JointStatus::kFault:
      effort_ = 0.0f;
      break;
  }
}

JointSnapshot JointController::snapshot() const {
  JointSnapshot s;
  s.position = pos_;
  s.velocity = vel_;
  s.effort = effort_;
  s.status = status_;
  s.fault = fault_;
  return s;
}
