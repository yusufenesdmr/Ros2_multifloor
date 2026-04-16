#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")
        self.declare_parameter("x", -5.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("topic", "/initialpose")
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("topic").value,
            10,
        )

    def publish_once(self):
        yaw = float(self.get_parameter("yaw").value)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.get_parameter("x").value)
        msg.pose.pose.position.y = float(self.get_parameter("y").value)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1

        for _ in range(10):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            time.sleep(0.5)


def main():
    rclpy.init(args=sys.argv)
    node = InitialPosePublisher()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.time() + 2.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)
    node.publish_once()
    deadline = time.time() + 1.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
