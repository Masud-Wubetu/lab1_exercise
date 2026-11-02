#!/usr/bin/python3
"""Action server that rotates the robot on the spot for a given angle, measured
with the wheel odometry, and returns the angle turned."""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Rotation about the vertical axis, in rad, of an orientation quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(target, current):
    """target - current, wrapped to [-pi, pi] so the difference is the short way."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")
        self.declare_parameter("turn_speed", 0.5)

        self.current_yaw = None
        self.accumulated_turn = 0.0
        self.lock = threading.Lock()

        group = ReentrantCallbackGroup()
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10, callback_group=group)
        self.action_server = ActionServer(
            self,
            Rotate,
            "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=group,
        )

    def on_odom(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self.lock:
            if self.current_yaw is not None:
                delta = angle_difference(yaw, self.current_yaw)
                self.accumulated_turn += delta
            self.current_yaw = yaw

    def on_goal(self, goal):
        if abs(goal.angle) < 1e-4 or self.current_yaw is None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        goal = goal_handle.request.angle
        with self.lock:
            start_accumulated = self.accumulated_turn

        turn_speed = abs(self.get_parameter("turn_speed").value)
        direction = 1.0 if goal > 0.0 else -1.0
        self.get_logger().info(f"Rotating {goal:.2f} rad at speed {turn_speed:.2f} rad/s")

        twist = Twist()
        twist.angular.z = direction * turn_speed

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        while rclpy.ok():
            with self.lock:
                turned = self.accumulated_turn - start_accumulated

            result.turned = float(turned)

            if goal_handle.is_cancel_requested:
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                self.get_logger().info(f"Canceled after turning {turned:.2f} rad")
                return result

            # Check if turned enough in the desired direction
            if (goal > 0.0 and turned >= goal) or (goal < 0.0 and turned <= goal):
                break

            self.cmd_vel.publish(twist)
            feedback.turned = float(turned)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        self.cmd_vel.publish(Twist())
        goal_handle.succeed()
        self.get_logger().info(f"Rotation reached, turned {result.turned:.2f} rad")
        return result


def main():
    rclpy.init()
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
