from typing import Dict, Union, Type
from ros_sugar.io import Topic
from ros_sugar.base_clients import ServiceClientHandler
from . import types
from . import clients


# Here we define the feedback types provided by this plugin
# All feedback types should be defined in the `types` module and should have a callback function defined there as well
# The keys of this dictionary should be the standard type names of the feedback types
# The values should be a Topic instance corresponding to the feedback topic the robot publishes
robot_feedback: Dict[str, Union[Topic, Type[ServiceClientHandler]]] = {
    # This defines a mapping between the standard "Pose" type (geometry_msgs) and the newly created JointStateMsg type
    # It also indicated that the robot publishes this feedback on the "joints_state" topic
    "Odometry": Topic(name="myrobot_odom", msg_type=types.RobotOdometry),
}


# Here we define the action types provided by this plugin
# All action types should be defined in the `types` module and should have a converter function defined there as well
# Or they should have a ServiceClientHandler defined in the `clients` module
# The keys of this dictionary should be the standard type names of the action types
# The values should be a Topic or a ServiceClientHandler instance corresponding to the action service the robot provides
robot_action: Dict[str, Union[Topic, Type[ServiceClientHandler]]] = {
    # This defines a mapping between the standard "Twist" type (geometry_msgs) and the JointCtrlClient service client
    "Twist": clients.CustomTwistClient
    # To define a mapping using a Topic instead of a ServiceClientHandler, you can uncomment the following line:
    # "Twist": Topic(name="myrobot_cmd_vel", msg_type=types.RobotTwist),
}


# ALL Sugarcoat compatible Robot Pugins should expose: robot_feedback and robot_action dicstionaries
__all__ = ["robot_feedback", "robot_action"]
