#!/usr/bin/env python3
"""
/cmd_vel → /rotav/cmd_vel köprüsü
Best-effort QoS ile Nav2'nin komutlarını robota iletir.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(Twist, "/rotav/cmd_vel", qos)
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, qos)
        self.get_logger().info("cmd_vel relay: /cmd_vel → /rotav/cmd_vel")

    def cb(self, msg: Twist):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
