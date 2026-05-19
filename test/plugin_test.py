"""End-to-end tests for the mixed-interface example ``MyRobotPlugin``.

Exercises the plugin against the mock robot from ``server_node.py`` across all
three of its transport families:

* **UDP** — telemetry decoded onto the feedback bus, ``Twist`` command sent.
* **ROS topic** — the battery feedback wired into a real ``BaseComponent`` via
  a native ROS subscription.
* **ROS service** — the ``dock`` action invoking a ``std_srvs/Trigger`` service.

Run with the example repo root on ``PYTHONPATH`` so ``myrobot_plugin`` imports,
the built ``myrobot_plugin_interface`` workspace sourced, and a Sugarcoat with
the ``ros_sugar.robot`` framework importable::

    python3 -m pytest test/plugin_test.py -v
"""

import os
import socket
import sys
import threading
import time

import pytest

# Make `myrobot_plugin` / `server_node` importable when run from the repo root.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402

from myrobot_plugin_interface.msg import CustomOdom  # noqa: E402
from ros_sugar.io.topic import Topic  # noqa: E402
from ros_sugar.robot import (  # noqa: E402
    InProcessFeedbackBus,
    RobotPlugin,
    RobotPluginHost,
    RosServiceTransport,
    RosTopicTransport,
    UdpTransport,
)

from myrobot_plugin import MyRobotPlugin, codecs  # noqa: E402
from server_node import MockRobot  # noqa: E402


class _MyRobotPluginForTest(MyRobotPlugin):
    """Test-only subclass overriding endpoints onto localhost free ports.

    The override pattern: set instance attributes *before* ``super().__init__()``
    so the production constructor reads them in place of the class defaults.
    """

    def __init__(self, *, telemetry_port: int, command_port: int):
        self.ROBOT_IP = "127.0.0.1"
        self.TELEMETRY_PORT = telemetry_port
        self.COMMAND_PORT = command_port
        super().__init__()


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture(scope="module")
def ros():
    """One rclpy context for the whole module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def mock_robot(ros):
    """A running mock robot (UDP threads + ROS node) plus a spinning executor
    that tests can add their own nodes to."""
    telemetry_port = _free_port()
    command_port = _free_port()
    robot = MockRobot(
        host="127.0.0.1",
        telemetry_port=telemetry_port,
        command_port=command_port,
        telemetry_rate_hz=50.0,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(robot)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(0.3)
    yield robot, telemetry_port, command_port, executor
    robot.stop()
    executor.shutdown()
    robot.destroy_node()
    thread.join(timeout=2.0)


# ---------------------------------------------------------------------------
# Construction / introspection (no robot needed)
# ---------------------------------------------------------------------------
def test_plugin_construction():
    """The plugin builds declaratively and spans three transport families."""
    plugin = MyRobotPlugin()
    assert plugin.metadata.name == "MyRobot"
    assert isinstance(plugin.transports["telemetry"], UdpTransport)
    assert isinstance(plugin.transports["commands"], UdpTransport)
    assert isinstance(plugin.transports["battery"], RosTopicTransport)
    assert isinstance(plugin.transports["dock"], RosServiceTransport)
    assert set(plugin.feedbacks) == {"Odometry", "Float32"}
    assert set(plugin.commands) == {"Twist"}
    assert {"stop", "dock"} <= set(plugin.actions.names())
    assert "low_battery" in plugin.events


def test_plugin_spec_roundtrip():
    """Production ``MyRobotPlugin()`` takes no kwargs and round-trips with an
    empty spec; an override subclass captures its overrides in the spec."""
    plugin = MyRobotPlugin()
    spec = plugin.to_spec()
    assert spec["class"].endswith(":MyRobotPlugin")
    assert spec["kwargs"] == {}
    rebuilt = RobotPlugin.from_spec(spec)
    assert set(rebuilt.feedbacks) == {"Odometry", "Float32"}

    # An override subclass captures its kwargs and applies the overrides as
    # instance attributes shadowing the class defaults.
    test_plugin = _MyRobotPluginForTest(telemetry_port=9990, command_port=9991)
    test_spec = test_plugin.to_spec()
    assert test_spec["class"].endswith(":_MyRobotPluginForTest")
    assert test_spec["kwargs"] == {"telemetry_port": 9990, "command_port": 9991}
    assert test_plugin.TELEMETRY_PORT == 9990


def test_plugin_introspection():
    """``describe`` reflects the plugin's mixed-transport surface."""
    desc = MyRobotPlugin().describe()
    transport_kinds = set(desc["transports"].values())
    assert transport_kinds == {"UdpTransport", "RosTopicTransport", "RosServiceTransport"}
    feedbacks = {f["key"]: f["transport_kind"] for f in desc["feedbacks"]}
    assert feedbacks == {"Odometry": "UdpTransport", "Float32": "RosTopicTransport"}
    assert {a["name"] for a in desc["actions"]} == {"stop", "dock"}


def test_codecs_roundtrip():
    """The UDP codec round-trips telemetry and commands."""
    assert codecs.decode_telemetry(codecs.encode_telemetry(1.0, 2.0, 0.5)) == (
        1.0,
        2.0,
        0.5,
    )
    assert codecs.decode_command(codecs.encode_command(0.1, 0.2, 0.3)) == (
        0.1,
        0.2,
        0.3,
    )
    assert codecs.decode_telemetry(codecs.encode_command(0, 0, 0)) is None


# ---------------------------------------------------------------------------
# UDP transport — feedback + command
# ---------------------------------------------------------------------------
def test_udp_feedback_and_command_flow(mock_robot):
    """The UDP telemetry stream is decoded onto the feedback bus and Twist
    commands are encoded and sent over UDP."""
    robot, telemetry_port, command_port, _executor = mock_robot
    plugin = _MyRobotPluginForTest(
        telemetry_port=telemetry_port, command_port=command_port
    )
    bus = InProcessFeedbackBus()
    host = RobotPluginHost(plugin, node=None, bus=bus)
    host.open()
    try:
        decoded = []
        bus.subscribe(
            "robot/feedback/Odometry",
            lambda data: decoded.append(deserialize_message(data, CustomOdom)),
        )
        deadline = time.time() + 2.0
        while not decoded and time.time() < deadline:
            time.sleep(0.02)
        assert decoded, "no UDP telemetry decoded onto the feedback bus"
        assert isinstance(decoded[0], CustomOdom)

        # A Twist command is encoded to UDP; the mock robot integrates it.
        twist = plugin.commands["Twist"]
        plugin.send_command(twist, twist.encoder([0.5, 0.0, 0.0]))
        time.sleep(0.4)
        assert robot._vx == pytest.approx(0.5), "robot did not receive the command"
    finally:
        host.close()


# ---------------------------------------------------------------------------
# ROS topic transport — battery feedback into a real component
# ---------------------------------------------------------------------------
def test_ros_topic_battery_feedback(mock_robot):
    """A ROS-topic feedback is wired into a BaseComponent via a native ROS
    subscription, not the feedback bus."""
    from ros_sugar.core.component import BaseComponent

    robot, telemetry_port, command_port, executor = mock_robot
    plugin = _MyRobotPluginForTest(
        telemetry_port=telemetry_port, command_port=command_port
    )
    plugin.set_bus(InProcessFeedbackBus())

    component = BaseComponent(
        component_name="battery_consumer",
        inputs=[Topic(name="battery_in", msg_type="Float32", use_plugin=True)],
    )
    component.rclpy_init_node()
    component._robot_plugin = plugin
    try:
        component._use_robot_plugin()

        # The Float32 input was remapped onto the robot's real ROS battery topic
        # and is NOT a bus-fed external topic.
        assert "myrobot/battery" in component.callbacks
        assert "battery_in" not in component.callbacks
        assert "myrobot/battery" not in component._external_topics

        component.create_all_subscribers()
        executor.add_node(component)
        callback = component.callbacks["myrobot/battery"]
        deadline = time.time() + 3.0
        while callback.msg is None and time.time() < deadline:
            time.sleep(0.05)
        assert callback.msg is not None, "no battery data over the ROS topic"
        assert 0.0 <= callback.get_output() <= 100.0
        executor.remove_node(component)
    finally:
        component.destroy_node()


# ---------------------------------------------------------------------------
# ROS service transport — dock action
# ---------------------------------------------------------------------------
def test_ros_service_dock_action(mock_robot):
    """The dock action invokes the robot's std_srvs/Trigger service."""
    robot, telemetry_port, command_port, executor = mock_robot
    plugin = _MyRobotPluginForTest(
        telemetry_port=telemetry_port, command_port=command_port
    )
    # The host runs the full bring-up: opens UDP transports, wires feedback,
    # then calls plugin.on_attached(node, bus) which binds the ROS-service
    # client. The dock action then calls the service through that bound client.
    host_node = rclpy.create_node("plugin_host_node")
    host = RobotPluginHost(plugin, node=host_node, bus=InProcessFeedbackBus())
    host.open()
    executor.add_node(host_node)
    try:
        assert robot.docked is False
        dock_action = plugin.actions.dock()
        dock_action()
        deadline = time.time() + 3.0
        while not robot.docked and time.time() < deadline:
            time.sleep(0.05)
        assert robot.docked is True, "dock service was not invoked"
        executor.remove_node(host_node)
    finally:
        host.close()
        host_node.destroy_node()
