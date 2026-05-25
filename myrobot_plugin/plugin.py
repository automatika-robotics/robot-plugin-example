"""The example robot plugin.

``MyRobotPlugin`` adapts a (made-up) robot with a **mixed** control surface to
Sugarcoat — the realistic case where a robot exposes different interfaces for
different things. It is a subclass of :class:`ros_sugar.robot.RobotPlugin` and
demonstrates the framework spanning three transport families in one plugin:

* **UDP** — a high-rate odometry telemetry stream (decoded from a binary wire
  format into the robot's custom ``CustomOdom`` message) and a low-latency
  ``Twist`` velocity command, plus a 2 Hz heartbeat.
* **ROS topic** — the robot's onboard computer already publishes its battery
  level on a plain ROS topic, so that feedback is wired with a
  :class:`~ros_sugar.robot.RosTopicTransport` and consumed via a native ROS
  subscription (no decoding needed).
* **ROS service** — a discrete "dock" behaviour is invoked through a
  :class:`~ros_sugar.robot.RosServiceTransport` calling a ``std_srvs/Trigger``
  service, exposed as a high-level plugin action.

``__init__`` stays declarative (no I/O, JSON-serializable kwargs); host-side
setup that needs an rclpy node (binding the ROS-service client) happens in
:meth:`on_attached`, the optional hook ``RobotPluginHost`` calls after wiring.
"""

import numpy as np
from std_srvs.srv import Trigger

# This plugin provides a kompass ``robot_config`` -- it's meant to run
# inside a kompass-based recipe, so kompass is a hard requirement. Fail
# fast with a clear message if it isn't installed. Plugins that don't
# target kompass can omit this whole block (and ``self.robot_config``).
try:
    from kompass.config import RobotConfig
    from kompass_core.models import AngularCtrlLimits, LinearCtrlLimits
except ImportError:
    import sys

    sys.stderr.write(
        "[myrobot_plugin] 'kompass' (and 'kompass-core') are required "
        "because this plugin provides a kompass robot_config. See the EMOS "
        "install guide: https://emos.automatikarobotics.com/getting-started/installation.html\n"
    )
    sys.exit(1)

from ros_sugar.core.action import Action
from ros_sugar.core.event import Event
from ros_sugar.robot import (
    ActionRegistry,
    EventRegistry,
    Feedback,
    PluginMetadata,
    RobotCommand,
    RobotPlugin,
    RosServiceTransport,
    RosTopicTransport,
    UdpTransport,
    plugin_action,
)
from ros_sugar.supported_types import Float32

# This would normally be the manufacturer's own ROS interface package.
from myrobot_plugin_interface.msg import CustomOdom

from . import codecs
from .types import RobotOdometry


def _decode_odom(raw: bytes):
    """UDP telemetry packet -> CustomOdom (or None for non-telemetry packets)."""
    parsed = codecs.decode_telemetry(raw)
    if parsed is None:
        return None
    msg = CustomOdom()
    msg.x, msg.y, msg.yaw = parsed
    return msg


def _encode_twist(output) -> bytes:
    """Component Twist output ``[vx, vy, vyaw]`` -> UDP command packet."""
    return codecs.encode_command(
        float(output[0]), float(output[1]), float(output[2])
    )


class MyRobotPlugin(RobotPlugin):
    """Example plugin for a robot with mixed UDP + ROS interfaces.

    Construction is zero-argument: ``MyRobotPlugin()`` — every endpoint and ROS
    name is part of *this robot's* identity, baked into the class. Override via
    a subclass when you need a non-default deployment (testing on localhost,
    custom subnet, a different battery topic on a refit unit)::

        class MyRobotOnLAN(MyRobotPlugin):
            ROBOT_IP = "192.168.1.42"        # control board's internal LAN IP
            BATTERY_TOPIC = "fleet/battery"  # alternate topic name on this unit
    """

    # --- Robot-specific endpoints ---
    #: IP of the board this plugin sends UDP to (on the robot's internal LAN).
    #: For the localhost example/mock this is ``127.0.0.1``; on a real robot
    #: with the control surface on a separate board this is that board's IP.
    ROBOT_IP = "127.0.0.1"
    #: Local UDP port the robot streams telemetry to.
    TELEMETRY_PORT = 9870
    #: Robot UDP port that accepts commands and heartbeats.
    COMMAND_PORT = 9871
    #: Local interface to bind the telemetry receiver on. ``0.0.0.0`` is the
    #: safe default — only worth pinning to a specific NIC IP for firewall scope.
    BIND_HOST = "0.0.0.0"
    #: ROS topic the robot publishes its battery level on.
    BATTERY_TOPIC = "myrobot/battery"
    #: ROS service that triggers the robot's docking routine.
    DOCK_SERVICE = "myrobot/dock"

    # --- Kompass robot model -------------------------------------------------
    # Strings so the class attrs don't depend on kompass being importable;
    # kompass's converters resolve them.
    ROBOT_DRIVE_TYPE = "DIFFERENTIAL_DRIVE"
    ROBOT_GEOMETRY_TYPE = "CYLINDER"
    #: ``[radius, height]`` in metres -- typical compact mobile base.
    ROBOT_GEOMETRY_PARAMS = (0.2, 0.4)
    #: Forward velocity limits.
    ROBOT_VX_MAX = 0.5
    ROBOT_VX_ACC = 1.0
    ROBOT_VX_DECEL = 1.5
    #: Angular velocity limits.
    ROBOT_OMEGA_MAX = 1.5
    ROBOT_OMEGA_ACC = 2.0
    ROBOT_OMEGA_DECEL = 2.0

    def __init__(self):
        self.metadata = PluginMetadata(
            name="MyRobot",
            vendor="Acme Robotics",
            version="2.0",
            description=(
                "An Acme Robotics MyRobot: a wheeled, differential-drive "
                "mobile base for indoor environments. It drives on the "
                "ground, reports its odometry and battery level, accepts "
                "velocity commands, and can drive itself onto a charging "
                "dock. It serves as the reference example for writing "
                "Sugarcoat robot plugins with mixed UDP and ROS interfaces."
            ),
        )

        # kompass-ready robot config -- sugarcoat's Launcher picks this up at
        # bringup and broadcasts to every kompass component. Recipes can
        # still override via ``launcher.robot = ...``.
        self.robot_config = RobotConfig(
            model_type=self.ROBOT_DRIVE_TYPE,
            geometry_type=self.ROBOT_GEOMETRY_TYPE,
            geometry_params=np.array(self.ROBOT_GEOMETRY_PARAMS),
            ctrl_vx_limits=LinearCtrlLimits(
                max_vel=self.ROBOT_VX_MAX,
                max_acc=self.ROBOT_VX_ACC,
                max_decel=self.ROBOT_VX_DECEL,
            ),
            ctrl_omega_limits=AngularCtrlLimits(
                max_vel=self.ROBOT_OMEGA_MAX,
                max_acc=self.ROBOT_OMEGA_ACC,
                max_decel=self.ROBOT_OMEGA_DECEL,
            ),
        )

        # --- UDP: recv-only telemetry stream + send-only command endpoint ---
        telemetry = UdpTransport(
            "telemetry", bind=(self.BIND_HOST, self.TELEMETRY_PORT)
        )
        commands = UdpTransport(
            "commands",
            send_to=(self.ROBOT_IP, self.COMMAND_PORT),
            keep_alive_fn=self._heartbeat,
            keep_alive_rate_hz=2.0,
        )
        # --- ROS topic: the robot already publishes battery level over ROS ---
        battery = RosTopicTransport(
            "battery", topic_name=self.BATTERY_TOPIC, msg_type=Float32
        )
        # --- ROS service: a discrete docking behaviour ---
        dock = RosServiceTransport(
            "dock", srv_name=self.DOCK_SERVICE, srv_type=Trigger
        )
        self.transports = {
            "telemetry": telemetry,
            "commands": commands,
            "battery": battery,
            "dock": dock,
        }

        self.feedbacks = {
            # UDP — decoded from the binary telemetry stream
            "Odometry": Feedback(
                key="Odometry",
                msg_type=RobotOdometry,
                transport=telemetry,
                decoder=_decode_odom,
                rate_hz=50.0,
                description="Base odometry, decoded from the UDP telemetry stream",
            ),
            # ROS topic — consumed via a native ROS subscription, no decoding
            "Float32": Feedback(
                key="Float32",
                msg_type=Float32,
                transport=battery,
                rate_hz=2.0,
                description="Battery level, read from the robot's ROS battery topic",
            ),
        }
        self.commands = {
            # UDP — a standard Twist encoded to the robot's wire format
            "Twist": RobotCommand(
                key="Twist",
                transport=commands,
                encoder=_encode_twist,
                description="Base velocity command, sent over UDP",
            )
        }
        self.actions = ActionRegistry(
            {
                "stop": self._make_stop_action,   # UDP — zero velocity
                "dock": self._make_dock_action,   # ROS service
            }
        )
        self.events = EventRegistry({"low_battery": self._make_low_battery_event})

    # -- host-side setup hook ------------------------------------------------
    def on_attached(self, node, bus) -> None:
        """Bind the ROS-service transport to the plugin's host node.

        ROS-service transports need an rclpy node to create their client; the
        :class:`~ros_sugar.robot.RobotPluginHost` hands the plugin one here.
        (Plugin actions run host-side, in the launcher process, so binding to
        the host node is enough.) When ``node`` is ``None`` — e.g. in tests
        that don't exercise the ROS-service action — the bind is skipped.
        """
        if node is None:
            return
        dock = self.transports["dock"]
        dock.bind_node(node)
        dock.open()

    # -- heartbeat -----------------------------------------------------------
    def _heartbeat(self) -> None:
        """Send the keep-alive packet the robot expects at 2 Hz."""
        self.transports["commands"].send(codecs.encode_heartbeat())

    # -- action / event factories -------------------------------------------
    @plugin_action(description="Command the robot to a full stop (zero velocity, UDP).")
    def _make_stop_action(self) -> Action:
        """Build an Action that commands the robot to a full stop (UDP)."""
        commands = self.transports["commands"]
        return Action(
            method=lambda: commands.send(codecs.encode_command(0.0, 0.0, 0.0))
        )

    @plugin_action(description="Trigger the robot's docking routine (ROS service).")
    def _make_dock_action(self) -> Action:
        """Build an Action that triggers the robot's docking routine (ROS service)."""
        dock = self.transports["dock"]
        return Action(method=lambda: dock.send(Trigger.Request()))

    def _make_low_battery_event(self, threshold: float = 20.0) -> Event:
        """Build an Event that fires when the battery drops below ``threshold``.

        The battery feedback is a ROS-topic feedback, so ``as_topic()`` resolves
        to the robot's real battery topic — the condition wires straight onto it.
        """
        battery = self.feedbacks["Float32"].as_topic()
        return Event(event_condition=battery.msg.data < threshold, on_change=True)
