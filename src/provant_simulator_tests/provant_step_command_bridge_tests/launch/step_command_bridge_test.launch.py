import os
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts


def _prepend_env_path(env_name: str, new_path: str) -> str:
    current = os.environ.get(env_name, "")
    if current:
        return new_path + os.pathsep + current
    return new_path


def generate_test_description():
    test_pkg_share = get_package_share_directory("provant_step_command_bridge_tests")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_plugins_prefix = get_package_prefix("provant_simulator_gz_plugins")

    world_path = os.path.join(
        test_pkg_share,
        "worlds",
        "step_command_bridge_minimal.sdf",
    )

    plugin_lib_path = os.path.join(gz_plugins_prefix, "lib")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_server.launch.py")
        ),
        launch_arguments={
            "world_sdf_file": world_path,
            "pause": "true",
            "on_exit_shutdown": "true",
        }.items(),
    )

    bridge = Node(
        package="provant_simulator_step_command_bridge",
        executable="provant_step_command_bridge",
        name="provant_step_command_bridge",
        output="screen",
        parameters=[
            {
                "world_name": "step_command_bridge_test",
                "timeout_ms": 3000,
            }
        ],
    )

    tester = Node(
        package="provant_step_command_bridge_tests",
        executable="step_command_bridge_tester_node",
        name="step_command_bridge_tester_node",
        output="screen",
        parameters=[
            {
                "total_requests": 5,
                "request_period_ms": 300,
                "startup_wait_ms": 1500,
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
                bridge,
                tester,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "tester": tester,
            "bridge": bridge,
            "gazebo": gazebo,
        },
    )


class TestStepCommandBridgeActive(unittest.TestCase):

    def test_tester_reports_progress_and_success(self, proc_output, tester):
        proc_output.assertWaitFor(
            "Published step request 1/5",
            process=tester,
            timeout=30.0,
        )
        proc_output.assertWaitFor(
            "Step request 5/5 succeeded",
            process=tester,
            timeout=30.0,
        )
        proc_output.assertWaitFor(
            "Test succeeded.",
            process=tester,
            timeout=30.0,
        )
