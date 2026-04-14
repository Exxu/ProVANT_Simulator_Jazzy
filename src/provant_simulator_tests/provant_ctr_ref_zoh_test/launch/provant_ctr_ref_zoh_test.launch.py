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
    test_pkg_share = get_package_share_directory("provant_ctr_ref_zoh_test")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_plugins_prefix = get_package_prefix("provant_simulator_gz_plugins")

    world_path = os.path.join(test_pkg_share, "worlds", "controlloop_minimal.sdf")
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
        parameters=[{"world_name": "temporal_loop_test", "timeout_ms": 3000}],
    )

    simulation_manager = Node(
        package="provant_simulator_simulation_manager",
        executable="provant_simulator_simulation_manager_node",
        name="simulation_manager",
        output="screen",
    )

    control_group = Node(
        package="provant_simulator_control_group_manager",
        executable="provant_simulator_control_group_manager",
        namespace="test_group",
        name="control_group_manager",
        output="screen",
        parameters=[{
            "control_step": 0.003,
            "has_disturbances": False,
            "has_estimator": False,
            "use_sim_time": True,
        }],
    )

    reference_generator = Node(
        package="provant_simulator_reference_generator",
        executable="ref_gen_node",
        namespace="test_group",
        name="reference_generator",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    state_vector_source = Node(
        package="provant_ctr_ref_zoh_test",
        executable="state_vector_source_node",
        namespace="test_group",
        name="state_vector_source_node",
        output="screen",
        parameters=[{"use_sim_time": True, "step_clock_topic": "step_clock", "state_size": 12}],
    )

    controller = Node(
        package="provant_ctr_ref_zoh_test",
        executable="test_controller_node",
        namespace="test_group",
        name="controller",
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[("reset", "controller/reset")],
    )

    control_zoh = Node(
        package="provant_simulator_zoh",
        executable="control_zoh_node",
        namespace="test_group",
        name="control_zoh",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "actuators": ["actuator_1"],
            "initial_value": [0.0],
        }],
    )

    tester = Node(
        package="provant_ctr_ref_zoh_test",
        executable="provant_ctr_ref_zoh_tester_node",
        name="provant_ctr_ref_zoh_tester_node",
        output="screen",
        parameters=[{
            "startup_wait_ms": 1500,
            "startup_after_pause_ms": 1500,
            "required_actuator_msgs": 7,
            "required_reference_msgs": 3,
            "control_period_steps": 3,
            "expected_step_dt_ns": 1000000,
            "time_tolerance_ns": 0,
            "world_name": "temporal_loop_test",
            "group_namespace": "/test_group",
            "actuator_topic": "actuator_1",
            "reference_tolerance": 1e-9,
            "effort_tolerance": 1e-9,
        }],
    )

    return (
        LaunchDescription([
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH",
                _prepend_env_path("GZ_SIM_PLUGIN_PATH", plugin_lib_path),
            ),
            gazebo,
            bridge,
            simulation_manager,
            control_group,
            reference_generator,
            state_vector_source,
            controller,
            control_zoh,
            tester,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"tester": tester},
    )


class TestCtrRefZohLoop(unittest.TestCase):
    def test_provant_ctr_ref_zoh_loop_reports_success(self, proc_output, tester):
        proc_output.assertWaitFor(
            expected_output="provant_ctr_ref_zoh_test succeeded.",
            process=tester,
            timeout=45.0,
        )
