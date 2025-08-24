"""Robot teleoperation with Joysick."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = "vizcc_controller"


def generate_launch_description():
    joystick_driver = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(
                get_package_share_directory(PACKAGE_NAME),
                "config",
                "joystick_config.yaml",
            )
        ],
    )

    joystick_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(
                get_package_share_directory(PACKAGE_NAME),
                "config",
                "joystick_teleop_config.yaml",
            )
        ],
    )
    return LaunchDescription([joystick_driver, joystick_teleop])
