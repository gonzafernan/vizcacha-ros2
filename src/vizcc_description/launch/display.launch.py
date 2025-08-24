"""
Visualize Vizcacha robot in RViz.

See https://docs.nav2.org/setup_guides/odom/setup_odom.html
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "vizcc_description"


def generate_launch_description():
    pkg_share = get_package_share_directory(PACKAGE_NAME)

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_share, "urdf", f"{PACKAGE_NAME}.urdf.xacro"),
        description="Absolute path to robot URDF file",
    )

    rviz_config_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(pkg_share, "rviz", "display.rviz"),
        description="Absolute path to RViz2 configuration file",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_config_arg,
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
