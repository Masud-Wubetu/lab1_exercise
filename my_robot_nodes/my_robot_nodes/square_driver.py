#!/usr/bin/python3
"""Action client that drives the robot in a square by calling /drive_distance and /rotate."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)

        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

    def run(self):
        self.get_logger().info("Waiting for /drive_distance action server...")
        if not self.drive_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("/drive_distance action server not available")
            return

        self.get_logger().info("Waiting for /rotate action server...")
        if not self.rotate_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("/rotate action server not available")
            return

        side_length = float(self.get_parameter("side_length").value)
        self.get_logger().info(f"Starting square of side length {side_length:.2f} m")

        for side in range(1, 5):
            self.get_logger().info(f"--- Side {side}/4: Driving forward {side_length:.2f} m ---")
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = side_length

            # Retry sending if simulation is still warming up (e.g., waiting for first odom)
            goal_handle = None
            for attempt in range(15):
                send_future = self.drive_client.send_goal_async(drive_goal)
                rclpy.spin_until_future_complete(self, send_future)
                goal_handle = send_future.result()
                if goal_handle and goal_handle.accepted:
                    break
                self.get_logger().info("Drive goal not accepted yet, waiting for odometry/simulation...")
                time.sleep(1.0)

            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(f"Drive goal rejected on side {side}")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            drive_status = result_future.result().status
            drive_result = result_future.result().result
            if drive_status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f"Drive did not succeed on side {side} (status {drive_status}, travelled {drive_result.travelled:.2f} m)"
                )
                return
            self.get_logger().info(f"Side {side}/4: drive succeeded, travelled {drive_result.travelled:.2f} m")

            self.get_logger().info(f"--- Side {side}/4: Turning 90 degrees left ---")
            rotate_goal = Rotate.Goal()
            rotate_goal.angle = math.pi / 2.0  # quarter turn left (~1.57 rad)

            send_future = self.rotate_client.send_goal_async(rotate_goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(f"Rotate goal rejected on side {side}")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            rotate_status = result_future.result().status
            rotate_result = result_future.result().result
            if rotate_status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f"Rotate did not succeed on side {side} (status {rotate_status}, turned {rotate_result.turned:.2f} rad)"
                )
                return
            self.get_logger().info(f"Side {side}/4: rotate succeeded, turned {rotate_result.turned:.2f} rad")

        self.get_logger().info("Square driving completed successfully!")


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
