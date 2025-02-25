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


def get_controller_nodes(
    use_mock_hardware, debug_hardware, use_sim_time, start_controller_node
):
    # load URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('bodenbot'),
                    'urdf',
                    'bodenbot.urdf.xacro',
                ]
            ),
            ' ',
            'use_mock_hardware:=',
            use_mock_hardware,
            ' ',
            'debug:=',
            debug_hardware,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('bodenbot'),
            'config',
            'controllers.yml',
        ]
    )

    nmea_config = PathJoinSubstitution(
        [
            FindPackageShare('bodenbot'),
            'config',
            'nmea_config.yml',
        ]
    )

    camera_config = PathJoinSubstitution(
        [
            FindPackageShare('bodenbot'),
            'config',
            'camera_config.yml',
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
            ('/bodenbot_controller/cmd_vel_unstamped', '/cmd_vel'),
        ],
        prefix=['gdb -ex run --args'],
        condition=IfCondition(start_controller_node),
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
        condition=IfCondition(start_controller_node),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        condition=IfCondition(start_controller_node),
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'bodenbot_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        condition=IfCondition(start_controller_node),
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    nmea_driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        parameters=[nmea_config],
        condition=UnlessCondition(use_mock_hardware),
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
        parameters=[camera_config],
        condition=UnlessCondition(use_mock_hardware),
    )

    # Controller nodes
    controller_nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        nmea_driver_node,
        zed_wrapper_node,
    ]

    return controller_nodes


def get_navigation_nodes(
    use_sim_time, start_navigation, start_traverse_layer, start_docking_server
):
    package_dir = get_package_share_directory('bodenbot')
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


def get_webots_nodes(use_sim_time, start_webots):
    webots_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('webots_dev'),
                    'launch',
                    'robot_launch.py',
                ]
            ),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'True',
        }.items(),
        condition=IfCondition(start_webots),
    )

    return [webots_cmd]


def generate_launch_description():
    declared_arguments = []

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='False',
            description='Run motor controller with mock hardware',
        )
    )

    start_controller_node = LaunchConfiguration('start_controller_node')
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_controller_node',
            default_value='True',
            description='Start controller node',
        )
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock if True',
        )
    )

    debug_hardware = LaunchConfiguration('debug_hardware')
    declared_arguments.append(
        DeclareLaunchArgument(
            'debug_hardware',
            default_value='False',
            description='Print dubugging info for hardware',
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

    start_webots = LaunchConfiguration('start_webots')
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_webots',
            default_value='False',
            description='Run in Webots',
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

    controller_nodes = get_controller_nodes(
        use_mock_hardware, debug_hardware, use_sim_time, start_controller_node
    )
    navigation_nodes = get_navigation_nodes(
        use_sim_time, start_navigation, start_traverse_layer, start_docking_server
    )
    webots_nodes = get_webots_nodes(use_sim_time, start_webots)

    # Create the launch description and populate
    return LaunchDescription(
        declared_arguments + controller_nodes + navigation_nodes + webots_nodes
    )

