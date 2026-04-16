#!/usr/bin/env python3
"""Publish zero joint states so RViz can render the full robot model."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "front_left_wheel_j",
    "front_right_wheel_j",
    "rear_left_wheel_j",
    "rear_right_wheel_j",
]


class FakeJointStatePublisher(Node):
    def __init__(self):
        super().__init__("fake_joint_state_publisher")
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_state)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [0.0] * len(JOINT_NAMES)
        msg.velocity = [0.0] * len(JOINT_NAMES)
        msg.effort = [0.0] * len(JOINT_NAMES)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
