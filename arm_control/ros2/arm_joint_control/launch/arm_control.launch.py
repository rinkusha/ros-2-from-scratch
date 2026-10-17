"""Brings up the joint bridge (serial link to the ESP32) and the trajectory planner.

    ros2 launch arm_joint_control arm_control.launch.py port:=/dev/ttyUSB0

Then:
    ros2 service call /joint_bridge/zero std_srvs/srv/Trigger      # at the home pose
    ros2 service call /joint_bridge/enable std_srvs/srv/Trigger
    ros2 topic pub -1 /joint_goal sensor_msgs/msg/JointState \
        "{name: [joint1, joint2], position: [0.5, -0.3]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("arm_joint_control")
    default_params = os.path.join(pkg, "config", "arm_joint_control.yaml")

    params_file = LaunchConfiguration("params_file")
    port = LaunchConfiguration("port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            Node(
                package="arm_joint_control",
                executable="joint_bridge",
                name="joint_bridge",
                output="screen",
                parameters=[params_file, {"port": port}],
            ),
            Node(
                package="arm_joint_control",
                executable="trajectory_planner",
                name="trajectory_planner",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
