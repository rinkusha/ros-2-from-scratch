# Control loop architecture

The position loop closes **on the ESP32 at 1 kHz**, reading the encoder
directly. ROS 2 only sends setpoints (100 Hz) and receives state (100 Hz).
The serial link is therefore outside the fast loop: its latency and jitter
affect how quickly a *new* target is picked up, not loop stability.

```
            ROS 2 on the laptop (WSL)                 │          ESP32 (ESP-IDF)
                                                      │
   joint_goal                                         │
       │                                              │
       ▼                                              │
 ┌─────────────────────┐                              │
 │ trajectory_planner  │  trapezoid profile per joint,│
 │  (100 Hz timer)     │  time-synchronised, limits   │
 │  pos + vel + acc    │  from max_velocity / max_acc │
 └──────────┬──────────┘                              │
            │ joint_setpoint (JointTrajectoryPoint)   │
            ▼                                         │
 ┌─────────────────────┐   SetpointMsg @ 100 Hz       │     ┌───────────────────────┐
 │    joint_bridge     │ ───────────────────────────────▶   │  comms task (core 0)  │
 │                     │   GainsMsg / CommandMsg +Ack │     │  COBS+CRC16 parser    │
 │  gains = ROS params │ ◀─────────────────────────────── ─ │  validates, acks      │
 │  services: enable,  │   StateMsg @ 100 Hz          │     └──────────┬────────────┘
 │  disable, zero,     │   LogMsg (ESP_LOG redirected)│                │ FreeRTOS queue
 │  clear_fault, ping  │                              │                ▼
 └──────────┬──────────┘   UART 921600 8N1            │     ┌───────────────────────┐
            │ joint_states (JointState)               │     │ control task (core 1) │◀─ esp_timer 1 kHz
            │ diagnostics (DiagnosticArray)           │     │                       │
            ▼                                         │     │  per joint, each 1 ms:│
     rviz / planner / your nodes                      │     │   encoder → pos, vel  │
                                                      │     │   watchdog (100 ms)   │
                                                      │     │   soft-limit check    │
                                                      │     │   PID + feed-forward  │
                                                      │     │   → motor command     │
                                                      │     └──┬────────────────▲───┘
                                                      │        │ LEDC PWM       │ PCNT quadrature
                                                      │        ▼                │ (hardware, x4)
                                                      │   ┌─────────┐      ┌────┴─────┐
                                                      │   │ H-bridge│─────▶│ Motor +  │
                                                      │   │         │      │ Encoder  │
                                                      │   └─────────┘      └──────────┘

   Outer loop: 100 Hz / 10 ms (setpoint stream)        Inner loop: 1 kHz / 1 ms (PID, on the MCU)
```

## PID (`arm_common/pid.hpp`, runs on the ESP32)

```
u = kp·(q_sp − q) + ki·∫(q_sp − q)dt − kd·q̇ + kff_v·q̇_sp + effort_ff
```

* **Derivative on measurement** (`− kd·q̇`), not on error: a setpoint step
  does not kick the output.
* **Velocity feed-forward** `kff_v·q̇_sp`: the planner's velocity does most of
  the tracking work; PID only corrects the residual.
* **Anti-windup**: the integral freezes while the output is saturated in the
  error's direction, and is clamped to `±i_limit`.
* **Output clamp** `±out_limit` (≤ 1.0 = full PWM duty).
* Measured velocity is a backward difference of the encoder count through a
  first-order low-pass (`vel_filter_hz`, default 50 Hz).

## State machine (per joint, firmware)

```
            enable               first setpoint
 DISABLED ──────────▶ HOLD ◀──────────────────── ENABLED
    ▲                  │   ───────────────────▶     │
    │ disable          │   no setpoint for 100 ms   │
    │◀─────────────────┴────────────────────────────┘
    │                                               │ |q| beyond soft limit + margin,
    │ clear_fault                                   │ or non-finite setpoint
    └─────────────── FAULT  ◀───────────────────────┘   (motor braked)
```

* `enable` is bumpless: the setpoint is set to the current position and the
  integrator is cleared, so the joint does not jump.
* `zero` (homing) is only accepted while disabled. Move the joint to its
  reference pose by hand (or against a stop), then `zero`.
* `HOLD` keeps the loop closed at the last measured position; it is what
  happens when the cable is pulled or the planner dies.

## Timing budget

| Stage | Rate | Where |
|---|---|---|
| Trajectory sample | 100 Hz | `trajectory_planner` wall timer |
| SetpointMsg over UART | 100 Hz, 77 B payload → ~85 B frame → 0.9 ms at 921600 baud | `joint_bridge` |
| Setpoint pick-up | next 1 ms step after the frame is parsed | comms → control queue |
| PID step | 1 kHz, measured `dt`, max period reported in `StateMsg.loop_period_max_us` | control task, core 1 |
| StateMsg over UART | 100 Hz, 99 B payload → ~108 B frame → 1.2 ms | comms task |

Total UART load is ~20 kB/s against ~92 kB/s available, so the link never
backs up. `joint_bridge` reports frame counts, setpoint age and loop jitter
on `/diagnostics`.

## What is deliberately not in the loop

* **No ros2_control yet.** The bridge/firmware split mirrors the hardware
  interface pattern (`write()` = SetpointMsg, `read()` = StateMsg), so wrapping
  it in a `hardware_interface::SystemInterface` later is mechanical.
* **No current sensing.** `kFaultOverCurrent` is reserved; add an ADC read in
  `JointController::step()` and `trip()` on it.
* **No gravity model.** `effort_ff` in SetpointMsg is the hook: compute a
  feed-forward torque on the ROS side and send it with the setpoint.
