// ESP32 joint controller: closes the position loop for each joint at 1 kHz
// and talks to the ROS 2 joint_bridge over UART. See ../../docs/control_loop.md.
#include "board_config.hpp"
#include "comms_task.hpp"
#include "control_task.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "serial_link.hpp"

namespace {
constexpr const char* kTag = "main";

SerialLink g_link;
ControlTask g_control;
CommsTask g_comms(g_link, g_control);
}  // namespace

extern "C" void app_main(void) {
  // Bring the link up first so every log line below reaches the host.
  ESP_ERROR_CHECK(g_link.init(cfg::kUartNum, cfg::kUartBaud, cfg::kUartTxPin, cfg::kUartRxPin));
  g_comms.installLogHook();
  ESP_LOGI(kTag, "esp32_joint_controller protocol v%u, %u joints, %u Hz loop",
           static_cast<unsigned>(arm::kProtocolVersion), static_cast<unsigned>(cfg::kNumJoints),
           static_cast<unsigned>(1000000 / cfg::kControlPeriodUs));

  ESP_ERROR_CHECK(g_control.init());
  ESP_ERROR_CHECK(g_control.start());
  ESP_ERROR_CHECK(g_comms.start());

  // app_main's task is no longer needed; the two workers run forever.
  vTaskDelete(nullptr);
}
