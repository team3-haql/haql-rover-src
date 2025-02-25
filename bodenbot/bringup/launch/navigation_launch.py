# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    boden_nav_dir = get_package_share_directory('boden_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    params_dir = os.path.join(boden_nav_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_params.yml")
    lattice_filepath = os.path.join(params_dir, "smac.json")

    param_substitutions = {
        'planner_server.ros__parameters.GridBased.lattice_filepath': lattice_filepath,
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites=param_substitutions, convert_types=True
    )

    container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": 'True',
            # 'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(container)
    ld.add_action(navigation2_cmd)

    return ld