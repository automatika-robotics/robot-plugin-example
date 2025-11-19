from myrobot_plugin_interface.srv import RobotActionCall

import rclpy
from rclpy.node import Node


class MinimalService(Node):
    def __init__(self):
        super().__init__("minimal_service")
        self.srv = self.create_service(
            RobotActionCall, "robot_control_service", self._service_callback
        )
        self.get_logger().info(
            "Robot Custom Server Node is Running!"
        )

    def _service_callback(self, request, response):
        response.success = True
        self.get_logger().info(f"Incoming request: vx: {request.vx}, vy: {request.vy}, omega: {request.vyaw}")

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
