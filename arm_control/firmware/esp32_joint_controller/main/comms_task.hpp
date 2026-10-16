// Host link task (core 0): parses frames from the UART, validates them, hands
// them to the control loop, answers with Acks, and forwards StateMsgs the
// control loop produces. Also installs an esp_log hook so ESP_LOGx output
// reaches the host as LogMsg frames instead of polluting the binary stream.
#pragma once

#include <cstdarg>
#include <cstddef>
#include <cstdint>

#include "arm_common/framing.hpp"
#include "arm_common/protocol.hpp"
#include "control_task.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "serial_link.hpp"

class CommsTask {
 public:
  CommsTask(SerialLink& link, ControlTask& control) : link_(link), control_(control) {}

  esp_err_t start();

  // Routes ESP_LOG output through the link. Safe to call before start().
  void installLogHook();

 private:
  static void taskEntry(void* arg);
  void run();
  void handleFrame();
  void ack(arm::MsgType type, uint16_t seq, arm::AckResult result);
  static int logVprintf(const char* fmt, va_list args);

  SerialLink& link_;
  ControlTask& control_;
  arm::FrameParser parser_;
  TaskHandle_t task_ = nullptr;
  uint16_t tx_seq_ = 0;

  static CommsTask* log_sink_;
};
