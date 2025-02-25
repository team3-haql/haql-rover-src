import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory("bodenbot")
    apriltag_params= os.path.join(package_dir, 'config', 'apriltag.yml')
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    # Launch the robot_state_publisher
    apriltag= Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[
            apriltag_params,
            use_sim_time,
        ],
        remappings=[
            ('/image_rect', '/zed2i/color/image_color'),
            ('/camera_info', '/zed2i/color/camera_info'),
        ],
    )

    docking_server = Node(
        package='bodenbot_scripts',
        executable='docking_server',
        name='docking_server',
        output='screen',
        parameters=[
            use_sim_time,
        ]
    )

    return LaunchDescription([apriltag, docking_server])


if __name__ == "__main__":
    generate_launch_description()
