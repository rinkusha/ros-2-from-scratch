#include "serial_link.hpp"

#include "esp_check.h"

namespace {
constexpr const char* kTag = "serial";
constexpr int kRxBufferBytes = 2048;
constexpr int kTxBufferBytes = 4096;
}  // namespace

esp_err_t SerialLink::init(uart_port_t port, int baud, int tx_pin, int rx_pin) {
  port_ = port;
  tx_mutex_ = xSemaphoreCreateMutex();
  if (tx_mutex_ == nullptr) return ESP_ERR_NO_MEM;

  uart_config_t cfg = {};
  cfg.baud_rate = baud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;

  ESP_RETURN_ON_ERROR(uart_driver_install(port_, kRxBufferBytes, kTxBufferBytes, 0, nullptr, 0), kTag, "install");
  ESP_RETURN_ON_ERROR(uart_param_config(port_, &cfg), kTag, "param");
  ESP_RETURN_ON_ERROR(uart_set_pin(port_, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), kTag, "pins");
  ready_ = true;
  return ESP_OK;
}

bool SerialLink::sendFrame(const void* payload, std::size_t len) {
  if (!ready_) return false;
  uint8_t frame[arm::kMaxFrameSize];
  const std::size_t n = arm::encodeFrame(payload, len, frame, sizeof(frame));
  if (n == 0) return false;

  if (xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(5)) != pdTRUE) return false;
  size_t free_bytes = 0;
  bool ok = false;
  if (uart_get_tx_buffer_free_size(port_, &free_bytes) == ESP_OK && free_bytes >= n) {
    ok = uart_write_bytes(port_, frame, n) == static_cast<int>(n);
  }
  xSemaphoreGive(tx_mutex_);
  return ok;
}

int SerialLink::read(uint8_t* buf, std::size_t cap, TickType_t timeout) {
  if (!ready_) return 0;
  const int n = uart_read_bytes(port_, buf, cap, timeout);
  return n < 0 ? 0 : n;
}
