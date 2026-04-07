from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="default",
        description="Gazebo world name used to build /world/<world_name>/control",
    )

    timeout_ms_arg = DeclareLaunchArgument(
        "timeout_ms",
        default_value="3000",
        description="Timeout in milliseconds for Gazebo world control requests",
    )

    world_name = LaunchConfiguration("world_name")
    timeout_ms = LaunchConfiguration("timeout_ms")

    step_command_bridge_node = Node(
        package="provant_simulator_step_command_bridge",
        executable="provant_step_command_bridge",
        name="provant_step_command_bridge",
        output="screen",
        parameters=[
            {
                "world_name": world_name,
                "timeout_ms": timeout_ms,
            }
        ],
    )

    return LaunchDescription(
        [
            world_name_arg,
            timeout_ms_arg,
            step_command_bridge_node,
        ]
    )