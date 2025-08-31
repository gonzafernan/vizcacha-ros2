"""Bringup vizcacha robot in simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("vizcc_description"),
            "launch",
            "gazebo.launch.py",
        )
    )
    controller_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("vizcc_controller"),
            "launch",
            "controller.launch.py",
        ),
    )
    joystick_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("vizcc_controller"),
            "launch",
            "joystick_teleop.launch.py",
        ),
    )
    return LaunchDescription([gazebo_launch, controller_launch, joystick_launch])
