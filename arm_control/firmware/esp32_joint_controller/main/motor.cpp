#include "motor.hpp"

#include <cmath>

#include "esp_check.h"

namespace {
constexpr const char* kTag = "motor";
constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;  // the only mode on S2/S3/C3

esp_err_t configureChannel(ledc_channel_t ch, gpio_num_t pin, ledc_timer_t timer) {
  ledc_channel_config_t c = {};
  c.gpio_num = pin;
  c.speed_mode = kSpeedMode;
  c.channel = ch;
  c.intr_type = LEDC_INTR_DISABLE;
  c.timer_sel = timer;
  c.duty = 0;
  c.hpoint = 0;
  return ledc_channel_config(&c);
}
}  // namespace

esp_err_t Motor::initTimer(ledc_timer_t timer) {
  ledc_timer_config_t t = {};
  t.speed_mode = kSpeedMode;
  t.duty_resolution = kResolution;
  t.timer_num = timer;
  t.freq_hz = cfg::kPwmFrequencyHz;
  t.clk_cfg = LEDC_AUTO_CLK;
  return ledc_timer_config(&t);
}

esp_err_t Motor::init(const cfg::JointPins& pins, ledc_channel_t first_channel, ledc_timer_t timer) {
  mode_ = pins.mode;
  invert_ = pins.invert_motor;
  ch_a_ = first_channel;
  ch_b_ = static_cast<ledc_channel_t>(first_channel + 1);

  ESP_RETURN_ON_ERROR(configureChannel(ch_a_, pins.motor_a, timer), kTag, "channel A");
  if (mode_ == cfg::DriverMode::kDualPwm) {
    ESP_RETURN_ON_ERROR(configureChannel(ch_b_, pins.motor_b, timer), kTag, "channel B");
  } else {
    dir_pin_ = pins.motor_b;
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << dir_pin_;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&io), kTag, "dir pin");
    gpio_set_level(dir_pin_, 0);
  }
  coast();
  return ESP_OK;
}

void Motor::writeDuty(ledc_channel_t ch, uint32_t duty) {
  if (duty > kMaxDuty) duty = kMaxDuty;
  ledc_set_duty(kSpeedMode, ch, duty);
  ledc_update_duty(kSpeedMode, ch);
}

void Motor::set(float u) {
  if (!std::isfinite(u)) u = 0.0f;
  if (u > 1.0f) u = 1.0f;
  if (u < -1.0f) u = -1.0f;
  last_u_ = u;
  if (invert_) u = -u;
  const uint32_t duty = static_cast<uint32_t>(std::fabs(u) * static_cast<float>(kMaxDuty) + 0.5f);

  if (mode_ == cfg::DriverMode::kPwmDir) {
    gpio_set_level(dir_pin_, u >= 0.0f ? 1 : 0);
    writeDuty(ch_a_, duty);
  } else {
    if (u >= 0.0f) {
      writeDuty(ch_b_, 0);
      writeDuty(ch_a_, duty);
    } else {
      writeDuty(ch_a_, 0);
      writeDuty(ch_b_, duty);
    }
  }
}

void Motor::coast() {
  last_u_ = 0.0f;
  writeDuty(ch_a_, 0);
  if (mode_ == cfg::DriverMode::kDualPwm) writeDuty(ch_b_, 0);
}

void Motor::brake() {
  last_u_ = 0.0f;
  if (mode_ == cfg::DriverMode::kDualPwm) {
    // IN1 = IN2 = high: slow-decay brake on most dual-input bridges.
    writeDuty(ch_a_, kMaxDuty);
    writeDuty(ch_b_, kMaxDuty);
  } else {
    // PWM/DIR drivers brake with PWM low; identical to coast on some boards.
    writeDuty(ch_a_, 0);
  }
}
