\
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
    test_pkg_share = get_package_share_directory("provant_temporal_loop_tests")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_plugins_prefix = get_package_prefix("provant_simulator_gz_plugins")

    world_path = os.path.join(
        test_pkg_share,
        "worlds",
        "temporal_loop_minimal.sdf",
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

    bridge = Node(
        package="provant_simulator_step_command_bridge",
        executable="provant_step_command_bridge",
        name="provant_step_command_bridge",
        output="screen",
        parameters=[
            {
                "world_name": "temporal_loop_test",
                "timeout_ms": 3000,
            }
        ],
    )

    simulation_manager = Node(
        package="provant_simulator_simulation_manager",
        executable="provant_simulator_simulation_manager_node",
        name="simulation_manager",
        output="screen",
    )

    tester = Node(
        package="provant_temporal_loop_tests",
        executable="temporal_loop_tester_node",
        name="temporal_loop_tester_node",
        output="screen",
        parameters=[
            {
                "total_cycles": 5,
                "request_period_ms": 300,
                "startup_wait_ms": 1500,
                "world_name": "temporal_loop_test",
                "control_timeout_ms": 3000,
                "stable_checks_required": 3,
                "group_namespace": "/test_group",
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
                simulation_manager,
                tester,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "tester": tester,
        },
    )


class TestTemporalLoopActive(unittest.TestCase):

    def test_temporal_loop_reports_success(self, proc_output, tester):
        proc_output.assertWaitFor(
            "Test succeeded.",
            process=tester,
            timeout=40.0,
        )
