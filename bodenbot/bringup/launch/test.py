import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def get_navigation_nodes(
    use_sim_time, start_navigation
):
    package_dir = get_package_share_directory('bodenbot')
    bringup_dir = get_package_share_directory('nav2_bringup')
    params_dir = os.path.join(package_dir, 'config')
    nav2_params = os.path.join(params_dir, 'nav2_params.yml')

    # load navgation parameters
    lattice_filepath = os.path.join(params_dir, 'smac.json')
    param_substitutions = {
        'planner_server.ros__parameters.GridBased.lattice_filepath': lattice_filepath,
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_navigation),
        prefix=['xterm -e gdb -ex run --args'],
    )
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': 'True',
            # 'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items(),
        condition=IfCondition(start_navigation),
    )

    return [
        container,
        navigation2_cmd,
    ]

def generate_launch_description():
    declared_arguments = []

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock if True',
        )
    )

    start_navigation = LaunchConfiguration('start_navigation')
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_navigation',
            default_value='True',
            description='Start navigation stack',
        )
    )

    navigation_nodes = get_navigation_nodes(
        use_sim_time, start_navigation
    )

    # Create the launch description and populate
    return LaunchDescription(
        declared_arguments + navigation_nodes
    )

