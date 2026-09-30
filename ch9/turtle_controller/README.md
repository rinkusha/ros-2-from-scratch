# turtle_controller: ROS 2 Training Material

A hands-on guide to three nodes in this package, using a turtle and a robot-arm joint as case studies.

| Node | File | Teaches |
|---|---|---|
| `turtle_controller` | `turtle_controller/turtle_controller.py` | Publishers, subscribers, parameters, services, async clients |
| `waypoint_server` | `turtle_controller/waypoint_server.py` | Writing a service server |
| `single_link` | `turtle_controller/single_link_lagrangian.py` | Lagrangian dynamics, timers, async service calls |

Custom interfaces live in `../my_robot_interfaces` (`ActivateTurtle.srv`, `GetNextWaypoint.srv`).

---

## Part 1: Case study, the turtle controller

**Scenario:** Drive a turtle in turtlesim. It moves in a circle, speeds up in the right half of the screen (x >= 5.5), and changes its pen color when it crosses the midline. A service lets an operator start or stop it.

### Architecture

```
/turtle1/pose ──► [turtle_controller] ──► /turtle1/cmd_vel
                      │      ▲
   /turtle1/set_pen ◄─┘      └── activate_turtle (service)
```

### Code walkthrough

**1. Parameters (configuration without code changes)**
```python
self.declare_parameter("color_1", [255, 0, 0])
self.declare_parameter("turtle_velocity", 1.0)
self.turtle_velocity_ = self.get_parameter("turtle_velocity").value
```
Declare a parameter with a default, then read it. Override at launch, e.g. `--ros-args -p turtle_velocity:=2.0`.

**2. Communication objects (created once in `__init__`)**
```python
self.set_pen_client_ = self.create_client(SetPen, "/turtle1/set_pen")
self.activate_turtle_service_ = self.create_service(ActivateTurtle, "activate_turtle", self.callback_activate_turtle)
self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
```
The node class owns all its communication. Outside code only creates the node. The `10` is the queue depth.

**3. The control loop is a subscription callback**
`callback_pose` runs on every pose message. It publishes a `Twist` (speed doubles when `pose.x >= 5.5`).

**4. State to detect a crossing**
`previous_x_` remembers which side of 5.5 the turtle was on. The pen color changes only when the side flips. Without this, you would call the service on every message.

**5. Asynchronous service call**
```python
future = self.set_pen_client_.call_async(request)
future.add_done_callback(self.callback_set_pen_response)
```
`call_async` returns immediately with a `Future`. The callback runs when the reply arrives, and the pose callbacks keep running in the meantime.

**6. Service server**
`callback_activate_turtle` sets `is_active_` and fills in `response.message`. It must return the response.

### Exercises
1. Change the boundary from 5.5 to a parameter named `boundary_x`.
2. Log the time between `call_async` and the response callback.
3. Call the service: `ros2 service call /activate_turtle my_robot_interfaces/srv/ActivateTurtle "{activate: false}"`.

### Known limitation (discussion point)
`call_set_pen` uses `while not wait_for_service(1.0)` inside a subscription callback. If turtlesim isn't running, this blocks the callback. Better: check `service_is_ready()` and skip or retry later, as `single_link` does below.

---

## Part 2: Case study, one-link motor arm (Lagrangian dynamics)

**Scenario:** A single link rotates about a joint driven by one motor. Compute the torque that moves it to a target angle, then ask a service for each new target.

### The physics
With joint angle `q`, link mass `m`, center-of-mass distance `lc`, and friction `b`:

- Kinetic energy: `T = 1/2 * I * qd^2`, where `I = m*lc^2 + J`
- Potential energy: `V = m*g*lc*sin(q)`
- Lagrangian: `L = T - V`
- Euler-Lagrange equation with motor torque `tau` and friction: `d/dt(dL/dqd) - dL/dq = tau - b*qd`

Result:
```
I*qdd + m*g*lc*cos(q) + b*qd = tau
```

### The controller (computed torque)
Choose a desired acceleration with PD feedback, then cancel gravity and friction:
```
qdd_des = Kp*e - Kd*qd
tau     = I*qdd_des + m*g*lc*cos(q) + b*qd
```
This makes the error behave like `e'' + Kd*e' + Kp*e = 0` (a damped second-order system). Torque is clamped to `tau_max`.

### Code map
| Piece | Where |
|---|---|
| Physics and controller | `SingleLinkLagrangian` (`compute_torque`, `step`, `at_target`) |
| 100 Hz loop | `create_timer(dt, self.update)` |
| Outputs | `/motor/angle`, `/motor/torque` (`Float64`) |
| Async request | `request_next_waypoint()` |
| Async response | `callback_waypoint_response()` |

### Async flow
```
update() @100 Hz ──► at_target()? ──yes──► request_next_waypoint()
                                              │ call_async (returns at once)
   arm keeps running, holding the old target  │
                                              ▼
                            callback_waypoint_response() sets q_target
```
`waypoint_pending_` prevents sending duplicate requests while one is outstanding.

### Server side
`waypoint_server.py` cycles through the `waypoints` parameter and returns `target_angle` and `success` for each request.

### Interface
```
# GetNextWaypoint.srv
float64 current_angle
---
float64 target_angle
bool success
```

### Run it
```bash
colcon build --packages-select my_robot_interfaces turtle_controller
source install/setup.bash

ros2 run turtle_controller waypoint_server
ros2 run turtle_controller single_link
ros2 topic echo /motor/angle
```
Custom waypoints: `ros2 run turtle_controller waypoint_server --ros-args -p waypoints:="[0.3, 1.2, -0.8]"`

### Exercises
1. Change `Kp` and `Kd`. Find values that overshoot, and values that are too slow.
2. Set `tau_max` to 1.0. What happens, and why (torque saturation vs. gravity load)?
3. Add a `timeout` so a stalled service does not leave `waypoint_pending_` stuck at `True`.
4. Set `b` to 0 in the plant but not the controller. Observe the mismatch (model error).

### Simplifications to be aware of
- Euler integration is simple but drifts. Use RK4 for better accuracy.
- The controller assumes an exact model. Real hardware needs identification and robustness.
- The timer runs on the node's clock, not a hard real-time loop.

---

## Concepts checklist
- [ ] Publisher, subscriber, and queue depth
- [ ] Parameters: declare, read, override
- [ ] Service server vs. client
- [ ] Sync vs. async calls, `Future`, done-callbacks
- [ ] Timers for fixed-rate control
- [ ] Callbacks share state, so guard against flooding (`waypoint_pending_`)
- [ ] Lagrangian modeling and computed-torque control
