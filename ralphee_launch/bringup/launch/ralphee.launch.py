import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def get_controller_nodes():
    # GPS Config
    nmea_config = PathJoinSubstitution(
        [
            FindPackageShare('ralphee_launch'),
            'config',
            'nmea_config.yml',
        ]
    )

    # GPS
    nmea_driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        parameters=[nmea_config]
    )

    camera_config = PathJoinSubstitution(
        [
            FindPackageShare('ralphee_launch'),
            'config',
            'camera_config.yml',
        ]
    )

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        name='zed_wrapper',
        output='screen',
        # prefix=['xterm -e valgrind --tools=callgrind'],
        # prefix=['xterm -e gdb -ex run --args'],
        # prefix=['gdbserver localhost:3000'],
        parameters=[camera_config]
    )

    # Start hardware
    ralphee_hardware_node = Node(
        package='ralphee_hardware',
        executable='hardware_controller'
    )

    # Controller nodes
    controller_nodes = [
        ralphee_hardware_node,
        nmea_driver_node,
        zed_wrapper_node,
    ]

    return controller_nodes


def get_navigation_nodes(
    use_sim_time, start_navigation, start_traverse_layer, start_docking_server
):
    package_dir = get_package_share_directory('ralphee_launch')
    traverse_layer = get_package_share_directory('traverse_layer')
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
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_navigation),
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

    traverse_layer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(traverse_layer, 'launch', 'traverse_layer_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'True',
        }.items(),
        condition=IfCondition(start_traverse_layer),
    )

    # Get the package directory
    apriltag_params = os.path.join(package_dir, 'config', 'apriltag.yml')

    # Launch the robot_state_publisher
    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[apriltag_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/image_rect', '/zed2i/color/image_color'),
            ('/camera_info', '/zed2i/color/camera_info'),
        ],
        condition=IfCondition(start_docking_server),
    )

    docking_server = Node(
        package='bodenbot_scripts',
        executable='docking_server',
        name='docking_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(start_docking_server),
    )

    return [
        container,
        navigation2_cmd,
        traverse_layer_cmd,
        apriltag,
        docking_server,
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

    start_traverse_layer = LaunchConfiguration('start_traverse_layer')
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_traverse_layer',
            default_value='True',
            description='Start Traversability Mapping',
        )
    )

    start_docking_server = LaunchConfiguration('start_docking_server')
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_docking_server',
            default_value='True',
            description='Start docking server',
        )
    )

    # Controls hardware
    controller_nodes = get_controller_nodes()
    # Calculates navigation
    navigation_nodes = get_navigation_nodes(
        use_sim_time, start_navigation, start_traverse_layer, start_docking_server
    )

    # Create the launch description and populate
    return LaunchDescription(
        declared_arguments + controller_nodes + navigation_nodes
    )

