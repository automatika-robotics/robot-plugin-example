"""SupportedType wrappers for the robot's custom ROS messages.

A manufacturer's custom feedback message is wrapped with
:func:`ros_sugar.robot.create_supported_type` and a ``callback`` that converts
the ROS message into a plain Python value the recipe can use. The component that
declares an ``Odometry`` input ends up reading the value this callback returns.
"""

import numpy as np

from ros_sugar.robot import create_supported_type

# This would normally be the manufacturer's own ROS interface package.
from myrobot_plugin_interface.msg import CustomOdom


def _odom_callback(msg: CustomOdom) -> np.ndarray:
    """Convert the robot's custom odometry message into ``[x, y, yaw]``."""
    return np.array([msg.x, msg.y, msg.yaw])


# Registered SupportedType wrapping CustomOdom. The plugin's Feedback decoder
# produces CustomOdom instances; this type's callback turns them into arrays.
RobotOdometry = create_supported_type(CustomOdom, callback=_odom_callback)
