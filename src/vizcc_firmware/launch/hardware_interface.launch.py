"""Launch vizcacha robot hardware interface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vizcc_description_pkg_share = get_package_share_directory("vizcc_description")
    vizcc_controller_pkg_share = get_package_share_directory("vizcc_controller")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    vizcc_description_pkg_share, "urdf", f"vizcc_description.urdf.xacro"
                ),
                " is_sim:=False",
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            os.path.join(
                vizcc_controller_pkg_share, "config", "vizcc_controllers.yaml"
            ),
        ],
    )
    return LaunchDescription([robot_state_publisher, controller_manager])
