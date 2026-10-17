// Quadrature encoder on the ESP32 PCNT peripheral (ESP-IDF 5.x pulse_cnt
// driver). Counting is done in hardware, x4 decoding, with the unit's
// accumulate-on-overflow flag so count() is a continuous 32-bit value.
#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

class Encoder {
 public:
  esp_err_t init(gpio_num_t pin_a, gpio_num_t pin_b, bool invert);

  // Accumulated count since the last zero().
  int32_t count() const;

  // Resets the hardware counter and the accumulator.
  esp_err_t zero();

  bool ok() const { return unit_ != nullptr; }

 private:
  pcnt_unit_handle_t unit_ = nullptr;
  pcnt_channel_handle_t chan_a_ = nullptr;
  pcnt_channel_handle_t chan_b_ = nullptr;
  bool invert_ = false;
};
