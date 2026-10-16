// Hardware configuration: edit this file to match your wiring.
//
// Default table is for a classic ESP32 (WROOM-32) driving 3 joints, which
// fits comfortably in the non-strapping GPIOs. For a 6-DOF arm either use an
// ESP32-S3 (more GPIOs; add three more rows) or run two boards with the
// joint_bridge started twice on different ports.
//
// Encoder inputs can use the input-only pins 34/35/36/39. Avoid the strapping
// pins 0, 2, 5, 12, 15 for anything that is driven at boot.
#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"

namespace cfg {

// ---- timing ----------------------------------------------------------------
constexpr uint32_t kControlPeriodUs = 1000;   // 1 kHz PID loop
constexpr uint32_t kStateDecimation = 10;     // StateMsg every 10 steps = 100 Hz
constexpr uint32_t kSetpointTimeoutUs = 100000;  // 100 ms without a setpoint -> HOLD

// ---- link ------------------------------------------------------------------
constexpr uart_port_t kUartNum = UART_NUM_0;  // USB-serial bridge on dev boards
constexpr int kUartBaud = 921600;
constexpr int kUartTxPin = UART_PIN_NO_CHANGE;
constexpr int kUartRxPin = UART_PIN_NO_CHANGE;

// ---- motor driver ----------------------------------------------------------
enum class DriverMode : uint8_t {
  kPwmDir,   // one PWM pin + one direction pin (e.g. DRV8876, many "PWM/DIR" boards)
  kDualPwm,  // two PWM pins IN1/IN2 (e.g. DRV8871, TB6612 with PWM on IN1/IN2, L298)
};

constexpr uint32_t kPwmFrequencyHz = 20000;  // above audible range

struct JointPins {
  gpio_num_t enc_a;
  gpio_num_t enc_b;
  gpio_num_t motor_a;      // PWM (kPwmDir) or IN1 (kDualPwm)
  gpio_num_t motor_b;      // DIR (kPwmDir) or IN2 (kDualPwm)
  DriverMode mode;
  float counts_per_rev;    // encoder counts per OUTPUT revolution, after x4
                           // quadrature decoding and gearbox: ppr * 4 * ratio
  bool invert_encoder;     // flip if positive command makes the count fall
  bool invert_motor;
};

// counts_per_rev example: a 16 PPR magnetic encoder on the motor shaft (64
// counts/rev after x4 decoding) behind a 30:1 gearbox -> 64 * 30 = 1920.
constexpr float kExampleCpr = 16.0f * 4 * 30;

#if CONFIG_IDF_TARGET_ESP32S3
// ESP32-S3: GPIO 0-21 and 26-48 exist (no 22-25). 19-20 are USB, 26-37 are
// typically flash/PSRAM on modules, 0/3/45/46 are strapping pins.
constexpr JointPins kJoints[] = {
    // enc_a      enc_b        motor_a      motor_b      mode                  cpr          inv_enc inv_mot
    {GPIO_NUM_4,  GPIO_NUM_5,  GPIO_NUM_8,  GPIO_NUM_9,  DriverMode::kPwmDir, kExampleCpr, false, false},
    {GPIO_NUM_6,  GPIO_NUM_7,  GPIO_NUM_10, GPIO_NUM_11, DriverMode::kPwmDir, kExampleCpr, false, false},
    {GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, DriverMode::kPwmDir, kExampleCpr, false, false},
    // Joints 4-6 (uncomment for a 6-DOF arm; the S3 has 4 PCNT units, so
    // joints 5-6 need the second board or a different encoder input):
    // {GPIO_NUM_1,  GPIO_NUM_2,  GPIO_NUM_12, GPIO_NUM_13, DriverMode::kPwmDir, kExampleCpr, false, false},
};
#else
// Classic ESP32 (WROOM-32). 34/35/36/39 are input-only: ideal for encoders.
constexpr JointPins kJoints[] = {
    // enc_a      enc_b        motor_a      motor_b      mode                  cpr          inv_enc inv_mot
    {GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_25, GPIO_NUM_26, DriverMode::kPwmDir, kExampleCpr, false, false},
    {GPIO_NUM_36, GPIO_NUM_39, GPIO_NUM_27, GPIO_NUM_14, DriverMode::kPwmDir, kExampleCpr, false, false},
    {GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_4,  GPIO_NUM_16, DriverMode::kPwmDir, kExampleCpr, false, false},
    // Joints 4-6 on the remaining non-strapping pins (8 PCNT units available):
    // {GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_17, GPIO_NUM_21, DriverMode::kPwmDir, kExampleCpr, false, false},
    // {GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_13, GPIO_NUM_15, DriverMode::kPwmDir, kExampleCpr, false, false},
    // {GPIO_NUM_2,  GPIO_NUM_12, GPIO_NUM_5,  GPIO_NUM_0,  DriverMode::kPwmDir, kExampleCpr, false, false},  // strapping pins: check boot
};
#endif

constexpr std::size_t kNumJoints = sizeof(kJoints) / sizeof(kJoints[0]);

// Position beyond a soft limit by more than this trips a fault.
constexpr float kSoftLimitMarginRad = 0.05f;

// Defaults used until the host pushes GainsMsg. Deliberately timid.
constexpr float kDefaultKp = 0.5f;
constexpr float kDefaultKd = 0.01f;
constexpr float kDefaultOutLimit = 0.5f;
constexpr float kDefaultVelFilterHz = 50.0f;

}  // namespace cfg
