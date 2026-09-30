#!/usr/bin/env python3
"""
Single-link, single-motor arm using Lagrangian dynamics.
Next target angle is fetched asynchronously from the get_next_waypoint service.

Model (link angle q measured from horizontal):
    T = 0.5 * I * qd^2                 kinetic energy
    V = m * g * lc * sin(q)            potential energy
    L = T - V
Euler-Lagrange: d/dt(dL/dqd) - dL/dq = tau - b*qd
    =>  I*qdd + m*g*lc*cos(q) + b*qd = tau
where I = m*lc^2 + J.

Control: computed torque with PD on the error.
    tau = I*(Kp*e - Kd*qd) + m*g*lc*cos(q) + b*qd
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from my_robot_interfaces.srv import GetNextWaypoint


class SingleLinkLagrangian:
    def __init__(self, m=0.5, lc=0.5, J=0.02, b=0.1, dt=0.01, tau_max=5.0):
        self.m, self.lc, self.b, self.dt, self.tau_max = m, lc, b, dt, tau_max
        self.I = m * lc ** 2 + J
        self.g = 9.81
        self.q = 0.0
        self.qd = 0.0
        self.q_target = 0.0
        self.Kp = 40.0
        self.Kd = 12.0

    def gravity_torque(self):
        return self.m * self.g * self.lc * math.cos(self.q)

    def compute_torque(self):
        e = math.atan2(math.sin(self.q_target - self.q), math.cos(self.q_target - self.q))
        qdd_des = self.Kp * e - self.Kd * self.qd
        tau = self.I * qdd_des + self.gravity_torque() + self.b * self.qd
        return max(-self.tau_max, min(self.tau_max, tau))

    def step(self):
        tau = self.compute_torque()
        qdd = (tau - self.gravity_torque() - self.b * self.qd) / self.I
        self.qd += qdd * self.dt
        self.q += self.qd * self.dt
        return tau

    def at_target(self, pos_tol=0.02, vel_tol=0.05):
        return abs(self.q_target - self.q) < pos_tol and abs(self.qd) < vel_tol


class SingleLinkNode(Node):
    def __init__(self):
        super().__init__("single_link_lagrangian")
        self.arm_ = SingleLinkLagrangian()
        self.waypoint_pending_ = False

        self.waypoint_client_ = self.create_client(GetNextWaypoint, "get_next_waypoint")
        self.angle_pub_ = self.create_publisher(Float64, "/motor/angle", 10)
        self.torque_pub_ = self.create_publisher(Float64, "/motor/torque", 10)
        self.timer_ = self.create_timer(self.arm_.dt, self.update)

    def request_next_waypoint(self):
        """Non-blocking: the control loop keeps running while we wait."""
        if self.waypoint_pending_ or not self.waypoint_client_.service_is_ready():
            return
        request = GetNextWaypoint.Request()
        request.current_angle = self.arm_.q
        self.waypoint_pending_ = True
        future = self.waypoint_client_.call_async(request)
        future.add_done_callback(self.callback_waypoint_response)

    def callback_waypoint_response(self, future):
        self.waypoint_pending_ = False
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Waypoint service call failed: {e}")
            return
        if response.success:
            self.arm_.q_target = response.target_angle
            self.get_logger().info(f"New target: {response.target_angle:.2f} rad")

    def update(self):
        tau = self.arm_.step()
        self.angle_pub_.publish(Float64(data=self.arm_.q))
        self.torque_pub_.publish(Float64(data=tau))

        # Arm holds its current target until a new one arrives.
        if self.arm_.at_target():
            self.request_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = SingleLinkNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
