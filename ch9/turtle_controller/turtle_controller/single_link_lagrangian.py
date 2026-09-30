#!/usr/bin/env python3
"""
Single-link, single-motor arm using Lagrangian dynamics.

Model (pendulum-style link, angle measured from horizontal):
    T = 0.5 * I * qd^2                 kinetic energy
    V = m * g * lc * sin(q)            potential energy
    L = T - V
Euler-Lagrange: d/dt(dL/dqd) - dL/dq = tau - b*qd
    =>  I*qdd + m*g*lc*cos(q) + b*qd = tau
where I = m*lc^2 + J (J = inertia about the center of mass).

Control: computed-torque (inverse dynamics) with PD on the error.
    tau = I*(Kp*e - Kd*qd) + m*g*lc*cos(q) + b*qd
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


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
        # Target joint angles in radians (replace with a waypoint service later)
        self.waypoints_ = [0.5, -0.5, 1.0, 0.0]
        self.idx_ = 0
        self.arm_.q_target = self.waypoints_[0]

        self.angle_pub_ = self.create_publisher(Float64, "/motor/angle", 10)
        self.torque_pub_ = self.create_publisher(Float64, "/motor/torque", 10)
        self.timer_ = self.create_timer(self.arm_.dt, self.update)

    def update(self):
        tau = self.arm_.step()
        self.angle_pub_.publish(Float64(data=self.arm_.q))
        self.torque_pub_.publish(Float64(data=tau))

        if self.arm_.at_target():
            self.idx_ = (self.idx_ + 1) % len(self.waypoints_)
            self.arm_.q_target = self.waypoints_[self.idx_]
            self.get_logger().info(f"Next target: {self.arm_.q_target:.2f} rad")


def main(args=None):
    rclpy.init(args=args)
    node = SingleLinkNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
