import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Find the grid_map_demos package share directory
    traverse_layer_dir = get_package_share_directory('traverse_layer')

    # Declare launch configuration variables that can access the launch arguments values
    filters_config_file = LaunchConfiguration('filters_config')
    visualization_config_file = LaunchConfiguration('visualization_config')

    # Declare launch arguments
    declare_filters_config_file_cmd = DeclareLaunchArgument(
        'filters_config',
        default_value=os.path.join(
            traverse_layer_dir, 'config', 'traverse_filters.yml'),
        description='Full path to the filter chain config file to use')

    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(
            traverse_layer_dir, 'config', 'visualization.yml'),
        description='Full path to the Gridmap visualization config file to use')

    # Declare node actions
    composable_nodes = ComposableNodeContainer(
        name='traverse_nodes',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='traverse_layer',
                plugin='traverse_layer::TraverseLayer',
                name='traverse_layer',
                parameters=[filters_config_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='traverse_layer',
                plugin='traverse_layer::PointcloudToGridmap',
                name='pointcloud_to_gridmap',
                parameters=[filters_config_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    local_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='local_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )

    global_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='global_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments to the launch description
    ld.add_action(declare_filters_config_file_cmd)
    ld.add_action(declare_visualization_config_file_cmd)

    # Add node actions to the launch description
    ld.add_action(composable_nodes)
    ld.add_action(local_map_visualization_node)
    ld.add_action(global_map_visualization_node)

    return ld