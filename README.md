# Example Robot Plugin

This package serves as an example of how to create a robot plugin compatible with the EmbodiedOS (EMOS)[https://automatikarobotics.com/emos/] or any (Sugarcoat)[https://automatika-robotics.github.io/sugarcoat/] based system.

It demonstrates how to define custom ROS2 interfaces (messages and services), map them to supported internal types, and expose them through a Python plugin structure.

This plugin example bridges standard robot commands (like `Twist`) and feedback (like `Odometry`) to custom hardware interfaces that can be defined by a robot manufacturer

See the steps to [create your own custom robot plugin based on this package](#4-creating-your-own-robot-plugin).

## Package Structure

```text
myrobot_plugin_interface/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Dependencies and metadata
├── msg/                    # Custom Message definitions, can be added here or can be imported from the manufacturer ROS2 interface package
│   ├── CustomOdom.msg
│   └── CustomTwist.msg
├── srv/                    # Custom Service definitions, can be added here or can be imported from the manufacturer ROS2 interface package
│   └── RobotActionCall.srv
├── myrobot_plugin/         # The Python plugin module
│   ├── __init__.py         # Plugin entry point
│   ├── clients.py          # Custom service client wrappers
│   └── types.py            # Type definitions and converters
└── server_node.py          # Example mock robot server to test publishing to a robot through a server call
```

## 1. Custom Interfaces

The plugin defines three ROS2 interfaces as an example of custom interfaces to communicate with the robot's ROS2 interface exposed by the manufacturer:

* **`CustomOdom.msg`**: A feedback message containing position (x, y, z) and orientation (pitch, roll, yaw).
* **`CustomTwist.msg`**: A command message for 2D velocity (vx, vy) and angular velocity (vyaw).
* **`RobotActionCall.srv`**: A service definition used to trigger actions on the robot, returning a success boolean.

## 2. Plugin Implementation

The core logic resides in the `myrobot_plugin` Python module.

### Supported Types (`types.py`)

This module defines how to translate between the custom robot types (`CustomOdom` and `CustomTwist`) and standard python types

* **Feedback (Callbacks):** Functions that convert incoming ROS messages into standard types.
    * *Example:* `_odom_callback` converts `CustomOdom` into a NumPy array `[x, y, yaw]`.
* **Actions (Converters):** Functions that convert standard commands (like `vx`, `vy`, `omega`) into custom ROS2 messages.
    * *Example:* `_ctr_converter` converts velocity inputs into a `CustomTwist` message.

```python
from ros_sugar.robot_plugin import create_supported_type
# Example: Creating a supported type for feedback
RobotOdometry = create_supported_type(CustomOdom, callback=_odom_callback)
```

### Service Clients (`clients.py`)

For robots that handle actions via ROS services (instead of topics), this module defines custom client wrappers inheriting from RobotPluginServiceClient.

* **CustomTwistClient**: Wraps the RobotActionCall service. It implements the _publish method to populate the service request (vx, vy, vyaw) and send it to the robot.

### Plugin Entry Point (`__init__.py`)

This file exposes the plugin capabilities to the framework using two specific dictionaries:

1. **robot_feedback**: Maps standard feedback names (e.g., "Odometry") to Topic instances using the custom types defined in types.py.

2. **robot_action**: Maps standard action names (e.g., "Twist") to either a Topic or a ServiceClientHandler (like CustomTwistClient).

```python
from . import types, clients

# Example configuration
robot_feedback = {
    "Odometry": Topic(name="myrobot_odom", msg_type=types.RobotOdometry),
}

robot_action = {
    "Twist": clients.CustomTwistClient
}
```

## 3. Testing

A `server_node.py` is provided to simulate the robot's ROS2 server. It spins a minimal node that listens to `robot_control_service` requests and logs the received velocity commands, allowing you to test the `CustomTwistClient` functionality.


## 4. Creating Your Own Robot Plugin

To create a new plugin for your own robot hardware, follow these steps using this package as a template:

0.  Optional: **Define Custom ROS Interfaces**
    If your robot's manufacturer-specific messages or services are not available to import from another package, define them in `msg/` and `srv/` folders.
    * *See:* `msg/CustomTwist.msg` and `srv/RobotActionCall.srv`.

1.  **Implement Type Converters (`types.py`)**
    Create a `types.py` module to handle data translation.
    * **For Each Feedback:** Define a callback function that transforms the custom ROS2 message into a standard Python type (like a NumPy array) which you can use directly in your system. Register it using `create_supported_type`.
    * **For Each Action:** Define a converter function that transforms standard Python inputs into the custom ROS2 message.

2.  **Handle Service Clients (`clients.py`)**
    If your robot actions require calling a ROS2 service, create a class inheriting from `RobotPluginServiceClient` in `clients.py`. Implement the `_publish` method to construct and send the service request.

3.  **Register the Plugin (`__init__.py`)**
    Expose your new capabilities in `__init__.py` by defining two dictionaries:
    * `robot_feedback`: Map standard names to `Topic` objects using your custom types.
    * `robot_action`: Map standard names to `Topic` objects (for topics) or Client classes (for services).

4.  **Configure the Build**
    Use the same `CMakeLists.txt` and `package.xml` for your new plugin package. Make sure to add any additional used dependencies.
