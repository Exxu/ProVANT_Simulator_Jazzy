#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _append_env_path(env_name: str, new_path: str) -> str:
    current = os.environ.get(env_name, "")
    if current:
      return new_path + os.pathsep + current
    return new_path


def generate_launch_description() -> LaunchDescription:
    test_pkg_share = get_package_share_directory("provant_step_manager_system_tests")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_plugins_prefix = get_package_prefix("provant_simulator_gz_plugins")

    world_path = os.path.join(
        test_pkg_share,
        "worlds",
        "step_manager_system_minimal.sdf",
    )

    plugin_lib_path = os.path.join(gz_plugins_prefix, "lib")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_server.launch.py")
        ),
        launch_arguments={
            "world_sdf_file": world_path,
            "on_exit_shutdown": "True",
        }.items(),
    )

    test_node = Node(
        package="provant_step_manager_system_tests",
        executable="test_step_manager_system_node",
        output="screen",
    )

    reset_world = ExecuteProcess(
        cmd=[
            "gz", "service",
            "-s", "/world/step_manager_test/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", "reset: {all: true}",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH",
                _append_env_path("GZ_SIM_PLUGIN_PATH", plugin_lib_path),
            ),
            gazebo,
            test_node,
            TimerAction(period=5.0, actions=[reset_world]),
        ]
    )