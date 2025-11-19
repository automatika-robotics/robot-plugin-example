# This module defines custom service clients for robot actions provided by this plugin
# Many robot actions are implemented as service clients by the manufacturers
# Here we define a custom service client for converting standard types into service requests and sending it to the robot using ROS2 services
from rclpy.node import Node
from ros_sugar.robot_plugin import RobotPluginServiceClient
from myrobot_plugin_interface.srv import RobotActionCall


# For each robot action that is implemented as a service client, define a custom service client class by inheriting from RobotPluginServiceClient
# The class should at least implement the `_publish` method to create and send the service request to the robot
# Any additional helper methods can be implemented as needed
class CustomTwistClient(RobotPluginServiceClient):
    """Class that implements a custom service client for sending the custom commands to the robot through a ROS2 service"""

    def __init__(self, client_node: Node, srv_name: str = "robot_control_service"):
        super().__init__(
            srv_type=RobotActionCall,
            srv_name=srv_name,
            client_node=client_node,
        )

    def _publish(self, vx, vy, omega, **_) -> bool:
        """In this `_publish` method, we create and send the custom service request"""
        req = RobotActionCall.Request()
        req.vx = vx
        req.vy = vy
        req.vyaw = omega
        response = self.send_request(req_msg=req)
        if response is not None:
            return response.success
        return False
