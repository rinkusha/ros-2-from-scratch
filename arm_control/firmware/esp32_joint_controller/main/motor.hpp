// H-bridge output on the LEDC PWM peripheral. Accepts a normalised command
// u in [-1, 1]; sign is direction, magnitude is duty.
#pragma once

#include <cstdint>

#include "board_config.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

class Motor {
 public:
  // `first_channel` and (for kDualPwm) `first_channel + 1` are used.
  esp_err_t init(const cfg::JointPins& pins, ledc_channel_t first_channel, ledc_timer_t timer);

  // Apply a command. Values outside [-1, 1] are clamped.
  void set(float u);

  // Outputs off: motor freewheels.
  void coast();

  // Both bridge legs low/high: windings shorted, motor resists motion.
  // Useful as a fault state on a gravity-loaded joint.
  void brake();

  float lastCommand() const { return last_u_; }

  // One-time timer setup shared by all motors (20 kHz, 10-bit).
  static esp_err_t initTimer(ledc_timer_t timer);

 private:
  void writeDuty(ledc_channel_t ch, uint32_t duty);

  cfg::DriverMode mode_ = cfg::DriverMode::kPwmDir;
  gpio_num_t dir_pin_ = GPIO_NUM_NC;
  ledc_channel_t ch_a_ = LEDC_CHANNEL_0;
  ledc_channel_t ch_b_ = LEDC_CHANNEL_1;
  bool invert_ = false;
  float last_u_ = 0.0f;

  static constexpr ledc_timer_bit_t kResolution = LEDC_TIMER_10_BIT;
  static constexpr uint32_t kMaxDuty = (1u << 10) - 1;
};
