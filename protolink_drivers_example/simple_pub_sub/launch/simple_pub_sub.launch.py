from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    return LaunchDescription(
        [
            log_level_arg,
            Node(
                namespace="protolink_drivers_example",
                package="simple_pub_sub",
                executable="simple_pub_sub",
                output="screen",
                ros_arguments=[
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )
