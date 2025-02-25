#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots soujourner driver."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.utils import controller_protocol, controller_ip_address


def generate_launch_description():
    package_dir = get_package_share_directory('webots_dev')
    mode = LaunchConfiguration('mode')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_foxglove = LaunchConfiguration('use_foxglove', default=False)
    show_gui = LaunchConfiguration('gui', default=False)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True,
        gui=show_gui,
    )

    print(os.environ)
    print(controller_protocol())
    print(controller_ip_address())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]


    robot_description_path = os.path.join(package_dir, 'resource', 'bodenbot_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    mappings = [
        ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
        ('/diffdrive_controller/odom', '/odometry/wheel'),
        ('/imu', '/imu_no_cov'),
        ('/gps', '/gps_point'),
        # remap camera stuff
        ('/Bodenbot/zed2i_color/camera_info', '/zed2i/color/camera_info'),
        ('/Bodenbot/zed2i_color/image_color', '/zed2i/color/image_color'),
        ('/Bodenbot/zed2i_range/camera_info', '/zed2i/range/camera_info'),
        ('/Bodenbot/zed2i_range/image', '/zed2i/range/image'),
        ('/Bodenbot/zed2i_range/point_cloud', '/zed2i/range/point_cloud'),
    ]
    bodenbot_driver = WebotsController(
        robot_name='Bodenbot',
        port='1234',
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True,
            },
            ros2_control_params
        ],
        remappings=mappings,
        # respawn=True
    )
    gps_pose = Node(
        package='webots_dev',
        executable='gps_pose_publisher',
        output='screen',
        name='gps_pose_publisher',
        parameters=[
            {
                'update_rate': 50,
                'use_sim_time': use_sim_time,
            }
        ]
    )
    
    # Wait for the simulation to be ready to start the tools and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=bodenbot_driver,
        nodes_to_start=ros_control_spawners+[gps_pose],
    )
    
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        condition=launch.conditions.IfCondition(use_foxglove),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='terrain.wbt',
            description='Choose one of the world files from `/webots_dev/worlds` directory',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Webots show gui',
        ),
        webots,
        webots._supervisor,
        
        robot_state_publisher,
        footprint_publisher,

        bodenbot_driver,
        waiting_nodes,
        foxglove_bridge,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
