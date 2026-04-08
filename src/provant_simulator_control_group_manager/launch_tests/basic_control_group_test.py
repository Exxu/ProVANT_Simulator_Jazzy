import rclpy
from rclpy.node import Node as ROSNode
from rclpy.time import Time as ROSTime
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_testing.actions import ReadyToTest
from launch_testing.markers import keep_alive

import pytest
import unittest
from threading import Event, Thread

from provant_simulator_interfaces.msg import StepClock


class MessageSubscriber:
    def __init__(self):
        self.msgs = []
        self.event = Event()

    def on_message(self, msg):
        self.msgs.append(msg)
        self.event.set()

    def clear_event(self):
        self.event.clear()


class ServiceCallback:
    def __init__(self):
        self.requests = []
        self.event = Event()

    def on_request(self, req, res):
        self.requests.append(req)
        self.event.set()
        return res

    def clear_event(self):
        self.event.clear()


class ControlManagerTestNode(ROSNode):
    def __init__(
        self,
        name: str = "test_node",
        namespace: str = "",
        has_estimator: bool = False,
        has_disturbances: bool = False,
    ):
        super().__init__(name, namespace=namespace)

        self.global_step_clock_pub = self.create_publisher(
            StepClock, "/provant_simulator/step_clock", 10
        )

        # Step clock subscriber
        self.step_clock = MessageSubscriber()
        self.step_clock_subscriber = self.create_subscription(
            StepClock, "step_clock", self.step_clock.on_message, 10
        )

        # ZOH Trigger subscriber
        self.zoh_trigger = MessageSubscriber()
        self.zoh_trigger_subscriber = self.create_subscription(
            EmptyMsg, "zoh_trigger", self.zoh_trigger.on_message, 10
        )

        # Reset service caller
        self.reset_client = self.create_client(EmptySrv, "reset")

        # Controller Reset service server
        self.controller_reset = ServiceCallback()
        self.controller_reset_server = self.create_service(
            EmptySrv, "controller/reset", self.controller_reset.on_request
        )

        # Control ZOH reset service server
        self.control_zoh_reset = ServiceCallback()
        self.control_zoh_reset_server = self.create_service(
            EmptySrv, "control_zoh/reset", self.control_zoh_reset.on_request
        )

        # Reference generator reset service
        self.ref_gen_reset = ServiceCallback()
        self.ref_gen_reset_server = self.create_service(
            EmptySrv, "ref_gen/reset", self.ref_gen_reset.on_request
        )

        # State estimator reset service
        self.has_estimator = has_estimator
        if has_estimator:
            self.estimator_reset = ServiceCallback()
            self.estimator_reset_service = self.create_service(
                EmptySrv, "estimator/reset", self.estimator_reset.on_request
            )

        # Disturbance generator reset service
        self.has_disturbances = has_disturbances
        if has_disturbances:
            self.dist_gen_reset = ServiceCallback()
            self.dist_gen_reset_server = self.create_service(
                EmptySrv, "dist_gen/reset", self.dist_gen_reset.on_request
            )
            self.dist_zoh_reset = ServiceCallback()
            self.dist_zoh_reset_server = self.create_service(
                EmptySrv, "dist_zoh/reset", self.dist_zoh_reset.on_request
            )

    def pub_global_step_clock(self, step: int, timestamp: ROSTime) -> None:
        msg = StepClock()
        msg.step = step
        msg.time = timestamp.to_msg()

        self.global_step_clock_pub.publish(msg)

    def call_reset(self):
        req = EmptySrv.Request()
        self.reset_client.call(req)


class BaseTestCase(unittest.TestCase):
    wait_time: float = 10.0

    def setUp(self) -> None:
        rclpy.init()
        self.node = ControlManagerTestNode(
            namespace="/provant_simulator/robot1",
            has_estimator=True,
            has_disturbances=True,
        )
        self.executor_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self.node,)
        )
        self.executor_thread.start()
        super().setUp()

    def tearDown(self) -> None:
        rclpy.shutdown()
        super().tearDown()

    def reset_manager(self):
        self.assertTrue(
            self.node.reset_client.wait_for_service(self.wait_time)
        )
        self.node.call_reset()

    def test_reset_server_available(self):
        self.assertTrue(
            self.node.reset_client.wait_for_service(self.wait_time)
        )

    def test_reset_calls_child_services(self):
        servers = [
            self.node.controller_reset,
            self.node.ref_gen_reset,
            self.node.control_zoh_reset,
        ]
        uncalled_servers = [
            self.node.dist_zoh_reset,
            self.node.dist_gen_reset,
            self.node.estimator_reset,
        ]

        for server in servers + uncalled_servers:
            server.requests.clear()
            server.clear_event()

        self.reset_manager()

        for server in servers:
            self.assertTrue(server.event.wait(self.wait_time))
            self.assertEqual(len(server.requests), 1)

        for server in uncalled_servers:
            self.assertFalse(server.event.wait(self.wait_time))
            self.assertEqual(len(server.requests), 0)


@pytest.mark.launch_test
@keep_alive
def generate_test_description() -> LaunchDescription:
    return LaunchDescription([
        LaunchNode(
            package="provant_simulator_control_group_manager",
            executable="provant_simulator_control_group_manager",
            namespace="/provant_simulator/robot1",
            name="control_group_manager",
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            parameters=[
                {'control_step': 0.001},
                {'has_estimator': False},
                {'has_disturbances': False},
            ],
        ),
        ReadyToTest(),
    ])
