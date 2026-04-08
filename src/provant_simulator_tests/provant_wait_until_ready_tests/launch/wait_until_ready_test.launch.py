import os
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions


def _prepend_env_path(env_name: str, new_path: str) -> str:
    current = os.environ.get(env_name, "")
    if current:
        return new_path + os.pathsep + current
    return new_path


def generate_test_description():
    test_pkg_share = get_package_share_directory("provant_wait_until_ready_tests")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_plugins_prefix = get_package_prefix("provant_simulator_gz_plugins")

    world_path = os.path.join(
        test_pkg_share,
        "worlds",
        "wait_until_ready_minimal.sdf",
    )

    plugin_lib_path = os.path.join(gz_plugins_prefix, "lib")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_server.launch.py")
        ),
        launch_arguments={
            "world_sdf_file": world_path,
            "on_exit_shutdown": "true",
        }.items(),
    )

    tester = Node(
        package="provant_wait_until_ready_tests",
        executable="wait_until_ready_tester_node",
        name="wait_until_ready_tester_node",
        output="screen",
        parameters=[
            {
                "startup_wait_ms": 300,
                "verification_window_ms": 500,
                "step_timeout_ms": 2000,
            }
        ],
    )

    return (
        LaunchDescription(
            [
                SetEnvironmentVariable(
                    "GZ_SIM_PLUGIN_PATH",
                    _prepend_env_path("GZ_SIM_PLUGIN_PATH", plugin_lib_path),
                ),
                gazebo,
                tester,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "tester": tester,
        },
    )


class TestWaitUntilReadyActive(unittest.TestCase):

    def test_wait_until_ready_reports_success(self, proc_output, tester):
        proc_output.assertWaitFor(
            "Test succeeded.",
            process=tester,
            timeout=30.0,
        )
