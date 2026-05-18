# Example Robot Plugin

This package shows how to write a **robot plugin** for [Sugarcoat](https://automatika-robotics.github.io/sugarcoat/)
(and the [EMOS](https://automatikarobotics.com/emos/) ecosystem built on it).

A robot plugin adapts a *specific* robot's control surface to Sugarcoat's
standard component I/O, without changing any component code. Real robots rarely
expose a single clean interface — they mix ROS topics, ROS services, vendor
UDP/TCP streams, and SDK callbacks. This example targets exactly that: a
(made-up) robot whose control surface spans **three transport families** in one
plugin.

## The robot's mixed control surface

| What | Transport | Why |
|:-----|:----------|:----|
| Odometry telemetry | **UDP** (`UdpTransport`) | high-rate binary stream from the robot's motion controller |
| Velocity command   | **UDP** (`UdpTransport`) | low-latency `Twist`, plus a 2 Hz heartbeat |
| Battery level       | **ROS topic** (`RosTopicTransport`) | the robot's onboard computer already publishes it on a ROS topic |
| Docking routine     | **ROS service** (`RosServiceTransport`) | a discrete behaviour invoked via `std_srvs/Trigger` |

The plugin presents all of this through one uniform `RobotPlugin` interface —
a component that subscribes to `Odometry` or `Float32`, or publishes a `Twist`,
is completely unaware which transport is behind it.

## The Plugin

`MyRobotPlugin` (in `myrobot_plugin/plugin.py`) is a subclass of
`ros_sugar.robot.RobotPlugin`. Its `__init__` is **declarative** — it constructs
transports and descriptors but performs no I/O — so the launcher can rebuild it
inside component subprocesses for multiprocess launch. Host-side setup that
needs an rclpy node (binding the ROS-service client) happens in the optional
`on_attached(node, bus)` hook, which `RobotPluginHost` calls once after wiring
is complete.

* **Transports** — two `UdpTransport`s (a receive-only telemetry endpoint and a
  send-only command endpoint with a heartbeat), one `RosTopicTransport` (the
  battery topic), one `RosServiceTransport` (the dock service).
* **Feedback** — `Odometry` is decoded from the UDP stream into the robot's
  `CustomOdom` message; `Float32` battery is consumed straight from the robot's
  ROS topic via a native subscription (no decoding).
* **Commands** — a standard `Twist` output is encoded to the UDP wire format.
* **Actions** — `plugin.actions.stop()` (UDP) and `plugin.actions.dock()`
  (ROS service).
* **Events** — `plugin.events.low_battery()`, built from a condition on the
  battery feedback — and because that feedback is a ROS-topic feedback, the
  condition wires straight onto the robot's real battery topic.

## Using the Plugin

`MyRobotPlugin` follows Sugarcoat's standard plugin contract: a **zero-argument
constructor** with every robot-specific endpoint baked in as a class attribute
(`ROBOT_IP`, `TELEMETRY_PORT`, `COMMAND_PORT`, `BIND_HOST`, `BATTERY_TOPIC`,
`DOCK_SERVICE`). The recipe author writes nothing about IPs, ports, or wire
formats:

```python
from ros_sugar.launch import Launcher
from myrobot_plugin import MyRobotPlugin

plugin = MyRobotPlugin()
launcher = Launcher(robot_plugin=plugin)
launcher.add_pkg(components=[...], multiprocessing=True)

# A plugin event (battery, over ROS) firing a plugin action (dock, over a ROS service)
launcher.on(plugin.events.low_battery(20.0), plugin.actions.dock())

launcher.bringup()
```

The launcher hosts the plugin, propagates it to every component, and tears it
down on shutdown. A component that declares an `Odometry` or `Float32` input
transparently receives the robot's data; a component that publishes a `Twist`
has its output encoded and sent over UDP. With `multiprocessing=True`,
Sugarcoat rebuilds the plugin in each component subprocess from a JSON spec
and fans telemetry out over a localhost socket — no extra configuration.

### Overriding for a specific unit

For testing on localhost, a non-default subnet, or a per-unit reconfigured
topic, **subclass** and override the class attributes:

```python
from myrobot_plugin import MyRobotPlugin

class MyRobotOnLAN(MyRobotPlugin):
    ROBOT_IP = "192.168.1.42"          # robot lives on a different subnet
    BATTERY_TOPIC = "fleet/battery"    # alternate topic name on this unit

plugin = MyRobotOnLAN()
```

## Testing

`server_node.py` is a mock robot with the same mixed surface — an `rclpy` node
(battery topic + dock service) that also runs UDP threads (telemetry +
commands):

```bash
python3 server_node.py --telemetry-port 9870 --command-port 9871
```

Inspect everything the plugin exposes as a JSON tree:

```bash
python3 -m ros_sugar.robot inspect myrobot_plugin:MyRobotPlugin
```

Run the test suite (mock robot + plugin, no hardware needed):

```bash
python3 -m pytest test/plugin_test.py -v
```

## Creating Your Own Robot Plugin

Use this package as a template:

1. **Custom interfaces (optional)** — if your robot has manufacturer-specific
   ROS messages, define them under `msg/`/`srv/` or import them from the
   manufacturer's interface package. A robot that is purely UDP/HTTP/SDK may
   need none.
2. **Codecs (`codecs.py`)** — implement the encode/decode for any non-ROS wire
   protocols your robot uses.
3. **Types (`types.py`)** — for each custom feedback message, wrap it with
   `create_supported_type` and a callback that converts it to a Python value.
4. **Plugin (`plugin.py`)** — subclass `RobotPlugin`; in a declarative,
   zero-arg `__init__` populate `transports`, `feedbacks`, `commands`,
   `actions` and `events`. Put robot-specific endpoints (IPs, ports, topic
   names) on the class as upper-case attributes so callers can override them
   by subclassing. Pick the transport per interface: `UdpTransport` /
   `HttpTransport` / `SdkCallbackTransport` for vendor links, `RosTopicTransport`
   / `RosServiceTransport` for interfaces the robot already exposes over ROS.
   Override the optional `on_attached(node, bus)` hook only when the plugin
   needs rclpy-node-dependent setup (e.g. binding a `RosServiceTransport`
   client to the host node).
5. **Entry point (`__init__.py`)** — export your `RobotPlugin` subclass.
6. **Build** — reuse this `CMakeLists.txt` / `package.xml`, adjusting package
   names and dependencies.

See the [Sugarcoat developer docs](https://automatika-robotics.github.io/sugarcoat/development/custom_robot_plugin.html)
for the full plugin contract.
