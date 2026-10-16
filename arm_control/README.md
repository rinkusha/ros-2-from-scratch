# arm_control — ROS 2 ↔ ESP32 joint control loop

Position control for a multi-joint robotic arm where the **PID runs on the
ESP32 at 1 kHz** and ROS 2 streams setpoints at 100 Hz. The architecture and
the reasons behind it are in [`docs/control_loop.md`](docs/control_loop.md).

```
arm_control/
├── ros2/
│   ├── arm_common/          header-only: wire protocol, COBS+CRC16 framing, PID, filters
│   │   └── test/            gtest: framing round-trips, PID behaviour on a simulated joint
│   └── arm_joint_control/   joint_bridge (serial ↔ topics/services), trajectory_planner
│       ├── launch/arm_control.launch.py
│       ├── config/arm_joint_control.yaml   gains, limits, port
│       └── test/            gtest: trapezoid profile + joint synchronisation
├── firmware/esp32_joint_controller/   ESP-IDF 5.x project (shares arm_common headers)
│   └── main/
│       ├── board_config.hpp   ← pins, encoder CPR, driver type: edit this first
│       ├── encoder.*          PCNT quadrature, x4, hardware counting
│       ├── motor.*            LEDC PWM, PWM/DIR or IN1/IN2 bridges
│       ├── joint_controller.* PID + state machine + soft limits
│       ├── control_task.*     1 kHz loop on core 1, setpoint watchdog, StateMsg
│       ├── comms_task.*       UART parser/acks on core 0, ESP_LOG → LogMsg
│       └── serial_link.*      UART driver wrapper
├── tools/esp32_console.py   bring-up without ROS (pyserial)
└── docs/control_loop.md
```

## Build: ROS 2 (Humble or Jazzy)

```bash
# in your colcon workspace
ln -s /path/to/ros-2-from-scratch/arm_control/ros2 src/arm_control
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to arm_joint_control
colcon test --packages-select arm_common arm_joint_control && colcon test-result --verbose
```

Only `ros2/` holds colcon packages; `firmware/` carries a `COLCON_IGNORE`.

## Build: firmware (ESP-IDF ≥ 5.1)

```bash
cd arm_control/firmware/esp32_joint_controller
idf.py set-target esp32          # or esp32s3
idf.py build flash               # the shared headers are picked up from ../../ros2/arm_common
```

Before flashing, open `main/board_config.hpp` and set the pin table, the
encoder counts per output revolution (`ppr × 4 × gear ratio`) and the driver
mode for each joint. The default table drives 3 joints on a classic ESP32.

## Bring-up (one joint at a time, arm unloaded)

1. `python3 tools/esp32_console.py /dev/ttyUSB0 watch` — you should see
   `StateMsg` lines at ~5 Hz and the firmware's boot log. Turn the joint by
   hand: `pos` must increase in the direction you call positive. If not, set
   `invert_encoder` for that joint.
2. `... gains 0 --kp 0.3 --kd 0.0 --out-limit 0.3` then `... enable` then
   `... step 0 0.2`. The joint must move *toward* 0.2 rad. If it runs away,
   set `invert_motor`. The `out_limit` keeps this safe.
3. Raise `kp` until it tracks crisply, add `kd` to kill overshoot, then a
   small `ki` for joints that sag under gravity. Watch `loop_max` stay near
   1000 µs.
4. `... disable`, move the joint to its reference pose, `... zero`.

## Run with ROS 2

```bash
ros2 launch arm_joint_control arm_control.launch.py port:=/dev/ttyUSB0
ros2 service call /joint_bridge/zero std_srvs/srv/Trigger       # at the home pose, while disabled
ros2 service call /joint_bridge/enable std_srvs/srv/Trigger
ros2 topic pub -1 /joint_goal sensor_msgs/msg/JointState "{name: [joint1, joint2], position: [0.5, -0.3]}"
ros2 topic echo /joint_states
ros2 param set /joint_bridge joint1.kp 1.5                        # pushed to the firmware live
```

Topics: `joint_goal` → planner → `joint_setpoint` → bridge → UART; UART →
bridge → `joint_states`, `diagnostics`. Services on `/joint_bridge`: `enable`,
`disable`, `zero`, `clear_fault`, `ping`; on `/trajectory_planner`: `stop`.

### WSL note

WSL 2 has no native USB. Attach the serial adapter with
[usbipd-win](https://github.com/dorssel/usbipd-win) (`usbipd attach --wsl
--busid <id>`); it shows up as `/dev/ttyUSB0` (CP210x/CH340) or
`/dev/ttyACM0` (native USB CDC). The 100 Hz outer loop tolerates the extra
latency; this is exactly why the PID is not on this side of the cable.

## Protocol in one paragraph

Every message is a packed struct from `arm_common/protocol.hpp`, followed by
CRC-16/CCITT-FALSE, COBS-encoded, terminated by `0x00`. Host → MCU:
`SetpointMsg` (positions, velocities, effort feed-forward for all joints),
`GainsMsg` (one joint), `CommandMsg` (enable/disable/zero/clear_fault/ping).
MCU → host: `StateMsg` (positions, velocities, efforts, status, fault bits,
setpoint age, loop jitter), `AckMsg` (for gains and commands), `LogMsg`.
Both builds include the same header, so a struct change is a protocol change
on both sides at once — bump `kProtocolVersion` when you make one.

## CI

`.github/workflows/arm_control_ci.yml` builds and tests the ROS 2 packages on
Humble and Jazzy containers and compiles the firmware with ESP-IDF 5.3 for
`esp32`. Hardware-in-the-loop is not covered; see the bring-up steps.
