import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    description_dir = get_package_share_directory("boden_description")

    # TODO: update to use new urdf file
    xacro_file = os.path.join(description_dir, "urdf", "bodenbot.urdf.xacro")

    # Launch the robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro",
                        " ",
                        xacro_file,
                    ]
                )
            }
        ],
        remappings=[
            (
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            )
        ],
    )

    # Launch the joint_state_publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    return LaunchDescription([robot_state_publisher_node, joint_state_publisher_node])


if __name__ == "__main__":
    generate_launch_description()
