#!/usr/bin/python3
"""React to obstacles by driving forward or turning away from them.

Subscribes to /obstacle, publishes /cmd_vel and /robot_status, and offers services
to enable/disable driving and set the turn speed.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        super().__init__("turn_away")
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("max_turn_speed", 1.5)

        self.enabled = False

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(RobotStatus, "robot_status", 10)

        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)
        self.create_service(SetBool, "turn_away_enable", self.on_enable)
        self.create_service(SetTurnSpeed, "set_turn_speed", self.on_set_turn_speed)

    def on_enable(self, request, response):
        if request.data:
            self.enabled = True
            response.success = True
            response.message = "turn_away enabled"
        else:
            self.enabled = False
            self.cmd_vel_pub.publish(Twist())
            response.success = True
            response.message = "turn_away disabled, stopped"
        self.get_logger().info(response.message)
        return response

    def on_set_turn_speed(self, request, response):
        max_turn_speed = self.get_parameter("max_turn_speed").value
        if request.turn_speed <= 0.0 or request.turn_speed > max_turn_speed:
            response.success = False
            response.message = f"turn speed must be above 0 and at most {max_turn_speed:.2f} rad/s"
        else:
            self.set_parameters([Parameter("turn_speed", value=float(request.turn_speed))])
            response.success = True
            response.message = f"turn speed set to {request.turn_speed:.2f} rad/s"
        self.get_logger().info(response.message)
        return response

    def on_obstacle(self, msg):
        forward_speed = self.get_parameter("forward_speed").value
        turn_speed = self.get_parameter("turn_speed").value

        status = RobotStatus()
        status.obstacle_distance = float(msg.distance)

        if not self.enabled:
            status.state = "stopped"
            self.status_pub.publish(status)
            return

        twist = Twist()
        if not msg.too_close:
            twist.linear.x = float(forward_speed)
            status.state = "forward"
        else:
            # angle is positive when the obstacle is to the left, so turn right then (negative angular.z)
            if msg.angle > 0.0:
                twist.angular.z = -float(turn_speed)
            else:
                twist.angular.z = float(turn_speed)
            status.state = "turning"

        self.cmd_vel_pub.publish(twist)
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
