// UART transport for the host link. Writes are mutex-protected so the comms
// task and the log hook can share the port; reads belong to the comms task.
#pragma once

#include <cstddef>
#include <cstdint>

#include "arm_common/framing.hpp"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class SerialLink {
 public:
  esp_err_t init(uart_port_t port, int baud, int tx_pin, int rx_pin);

  // Frames `payload` and writes it. Returns false if the TX buffer is full
  // (the frame is dropped rather than blocking the caller).
  bool sendFrame(const void* payload, std::size_t len);

  template <typename T>
  bool send(const T& msg) {
    return sendFrame(&msg, sizeof(T));
  }

  // Reads raw bytes with a timeout. Returns bytes read (0 on timeout).
  int read(uint8_t* buf, std::size_t cap, TickType_t timeout);

  bool ready() const { return ready_; }

 private:
  uart_port_t port_ = UART_NUM_0;
  SemaphoreHandle_t tx_mutex_ = nullptr;
  bool ready_ = false;
};
