#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import GetNextWaypoint


class WaypointServerNode(Node):
    """Serves the next target joint angle (rad), cycling through a list."""

    def __init__(self):
        super().__init__("waypoint_server")
        self.declare_parameter("waypoints", [0.5, -0.5, 1.0, 0.0])
        self.waypoints_ = list(self.get_parameter("waypoints").value)
        self.idx_ = 0
        self.server_ = self.create_service(
            GetNextWaypoint, "get_next_waypoint", self.callback_get_next_waypoint)
        self.get_logger().info("Waypoint server ready.")

    def callback_get_next_waypoint(self, request: GetNextWaypoint.Request,
                                   response: GetNextWaypoint.Response):
        response.target_angle = float(self.waypoints_[self.idx_])
        response.success = True
        self.get_logger().info(
            f"Current {request.current_angle:.2f} rad -> next {response.target_angle:.2f} rad")
        self.idx_ = (self.idx_ + 1) % len(self.waypoints_)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
