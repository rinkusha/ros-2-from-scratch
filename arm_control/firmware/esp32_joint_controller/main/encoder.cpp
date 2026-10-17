#include "encoder.hpp"

#include "esp_check.h"

namespace {
constexpr const char* kTag = "encoder";
constexpr int kHighLimit = 32767;
constexpr int kLowLimit = -32768;
}  // namespace

esp_err_t Encoder::init(gpio_num_t pin_a, gpio_num_t pin_b, bool invert) {
  invert_ = invert;

  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.high_limit = kHighLimit;
  unit_cfg.low_limit = kLowLimit;
  unit_cfg.flags.accum_count = 1;  // keep counting across the 16-bit limits
  ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &unit_), kTag, "pcnt_new_unit");

  pcnt_glitch_filter_config_t filter = {};
  filter.max_glitch_ns = 1000;
  ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(unit_, &filter), kTag, "glitch filter");

  // Channel A counts edges on A, direction from the level of B; channel B the
  // mirror image. Together they give x4 quadrature decoding.
  pcnt_chan_config_t a_cfg = {};
  a_cfg.edge_gpio_num = pin_a;
  a_cfg.level_gpio_num = pin_b;
  ESP_RETURN_ON_ERROR(pcnt_new_channel(unit_, &a_cfg, &chan_a_), kTag, "channel A");
  pcnt_chan_config_t b_cfg = {};
  b_cfg.edge_gpio_num = pin_b;
  b_cfg.level_gpio_num = pin_a;
  ESP_RETURN_ON_ERROR(pcnt_new_channel(unit_, &b_cfg, &chan_b_), kTag, "channel B");

  ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_a_, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE),
                      kTag, "edge A");
  ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_a_, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                      kTag, "level A");
  ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_b_, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                   PCNT_CHANNEL_EDGE_ACTION_DECREASE),
                      kTag, "edge B");
  ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_b_, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                      kTag, "level B");

  // Watch points at the limits are what make accum_count work.
  ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(unit_, kHighLimit), kTag, "watch high");
  ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(unit_, kLowLimit), kTag, "watch low");

  ESP_RETURN_ON_ERROR(pcnt_unit_enable(unit_), kTag, "enable");
  ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unit_), kTag, "clear");
  ESP_RETURN_ON_ERROR(pcnt_unit_start(unit_), kTag, "start");
  return ESP_OK;
}

int32_t Encoder::count() const {
  int value = 0;
  if (unit_ == nullptr || pcnt_unit_get_count(unit_, &value) != ESP_OK) return 0;
  return invert_ ? -static_cast<int32_t>(value) : static_cast<int32_t>(value);
}

esp_err_t Encoder::zero() {
  if (unit_ == nullptr) return ESP_ERR_INVALID_STATE;
  return pcnt_unit_clear_count(unit_);
}
