# This module defines custom supported types for all robot feedback (subscribed topics) and all robot actions through published topics
# A types that will be used as a feedback should have a "callback" function defined here
# This callback function will be used to convert the incoming ROS message into a standard python type or any other supported type
# A type that will be used as an action should have a "converter" function defined here
# This converter function will be used to convert the standard python type or any other supported type into a ROS message for publishing
import numpy as np
from ros_sugar.supported_types import create_supported_type
# These can be the manufacturer's custom message types
from myrobot_plugin_interface.msg import CustomOdom
from myrobot_plugin_interface.msg import CustomTwist


# EXAMPLE 1: Defining a supported type for feedback (subscription)
# Define Odom callback (for subscription)
def _odom_callback(msg: CustomOdom, **_) -> np.ndarray:
    if not msg:
        return
    return np.array(
        [
            msg.x,
            msg.y,
            msg.yaw,
        ]
    )  # x, y, yaw


# After defining the callback, we create the supported type by using the create_supported_type function from ros_sugar
RobotOdometry = create_supported_type(CustomOdom, callback=_odom_callback)


# EXAMPLE 2: Defining a supported type for action (publishing)
# Define Twist converter (for publishing)
def _ctr_converter(
    vx: float, vy: float, omega: float, **_
) -> CustomTwist:
    msg = CustomTwist()
    msg.vx = vx
    msg.vy = vy
    msg.vyaw = omega
    return msg


RobotTwist = create_supported_type(
    CustomTwist,
    converter=_ctr_converter,
)
