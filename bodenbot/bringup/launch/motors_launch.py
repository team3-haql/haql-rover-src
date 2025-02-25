#!/usr/bin/env python

"""Launch motor driver."""

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("boden_misc")

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    config_file = LaunchConfiguration("motor_driver_config")

    motor_driver = Node(
        package="boden_misc",
        executable="motor_driver",
        output="screen",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motor_driver_config",
                default_value=os.path.join(package_dir, "config", "motor_driver.yml"),
                description="Motor driver configuration.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use sim time",
            ),
            motor_driver,
        ]
    )
