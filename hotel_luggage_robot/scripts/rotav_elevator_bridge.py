#!/usr/bin/env python3
"""
rotav_elevator_bridge.py
========================
Köprü nodu: rotav_sim asansör topic'lerini hotel_luggage_robot
wormhole_navigator'ın beklediği formata çevirir.

rotav_sim → hotel_luggage_robot dönüşüm tablosu:
  /elevator/state (String)
    "IDLE"         → /elevator_controller/door_state "closed"
    "DOOR_OPENING" → /elevator_controller/door_state "open"
    "WAITING_ENTRY"→ /elevator_controller/door_state "open"
    "DOOR_CLOSING" → /elevator_controller/door_state "closed"
    "MOVING"       → /elevator_controller/door_state "in_transit"
    "DOOR_OPEN_EXIT"→ /elevator_controller/door_state "open"
    "WAITING_EXIT" → /elevator_controller/door_state "open"
    "DOOR_CLOSE_DONE"→/elevator_controller/door_state "closed"

Servis köprüsü:
  /elevator_controller/call_elevator (CallElevator)
    → /elevator/floor_request (Int8)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState

try:
    from hotel_luggage_robot.srv import CallElevator
    _HAS_HOTEL_MSGS = True
except ImportError:
    _HAS_HOTEL_MSGS = False
    from std_srvs.srv import Trigger

# rotav_sim durum → hotel format eşlemesi
STATE_MAP = {
    "IDLE":            "closed",
    "DOOR_OPENING":    "open",
    "WAITING_ENTRY":   "open",
    "DOOR_CLOSING":    "closed",
    "MOVING":          "in_transit",
    "DOOR_OPEN_EXIT":  "open",
    "WAITING_EXIT":    "open",
    "DOOR_CLOSE_DONE": "closed",
}

FLOOR_NAME_TO_INT = {
    "floor_1": 1,
    "floor_2": 2,
    "floor1": 1,
    "floor2": 2,
    "floor_3": 3,
    "floor_4": 4,
    "floor_5": 5,
}

FLOOR_TARGET_POSES = {
    1: {"x": 4.80, "y": 0.0, "z": 0.075, "yaw": math.pi},
    2: {"x": 4.80, "y": 0.0, "z": 3.575, "yaw": math.pi},
}


class RotavElevatorBridge(Node):
    def __init__(self):
        super().__init__("rotav_elevator_bridge")

        # rotav_sim asansör durumunu al
        self.create_subscription(
            String, "/elevator/state",
            self._state_cb, 10)

        # hotel_luggage_robot formatında yayınla
        self._door_pub = self.create_publisher(
            String, "/elevator_controller/door_state", 10)

        # Kat isteklerini ilet
        self._floor_req_pub = self.create_publisher(
            Int8, "/elevator/floor_request", 10)
        self._set_state_cli = self.create_client(
            SetEntityState, "/set_entity_state")

        # CallElevator servisini sağla
        if _HAS_HOTEL_MSGS:
            self.create_service(
                CallElevator,
                "/elevator_controller/call_elevator",
                self._call_elevator_cb)
            self.get_logger().info(
                "CallElevator servisi aktif (hotel_luggage_robot msgs)")
        else:
            self.get_logger().warn(
                "hotel_luggage_robot msgs bulunamadı – servis devre dışı")

        self._last_state = ""
        self.get_logger().info(
            "rotav_elevator_bridge hazır\n"
            "  /elevator/state → /elevator_controller/door_state")

    def _state_cb(self, msg: String):
        rotav_state = msg.data.strip()
        hotel_state = STATE_MAP.get(rotav_state, "closed")

        if hotel_state != self._last_state:
            self.get_logger().info(
                f"Asansör: {rotav_state} → {hotel_state}")
            self._last_state = hotel_state

        self._door_pub.publish(String(data=hotel_state))

    def _call_elevator_cb(self, request, response):
        target_floor_name = request.target_floor
        floor_int = FLOOR_NAME_TO_INT.get(target_floor_name, 1)

        self.get_logger().info(
            f"Asansör çağrısı: {request.current_floor} → "
            f"{target_floor_name} (Gazebo kat {floor_int})")

        self._floor_req_pub.publish(Int8(data=floor_int))
        threading.Thread(
            target=self._teleport_robot_after_delay,
            args=(floor_int,),
            daemon=True,
        ).start()

        response.success       = True
        response.elevator_id   = "rotav_elevator"
        response.wait_time_sec = 15.0
        response.message       = f"Asansör kat {floor_int}'e gidiyor"
        return response

    def _teleport_robot_after_delay(self, floor_int: int):
        pose = FLOOR_TARGET_POSES.get(floor_int)
        if pose is None:
            self.get_logger().warn(f"Teleport konumu tanimsiz: kat {floor_int}")
            return

        time.sleep(2.0)
        if not self._set_state_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("/set_entity_state hazir degil")
            return

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = "rotav"
        req.state.reference_frame = "world"
        req.state.pose.position.x = pose["x"]
        req.state.pose.position.y = pose["y"]
        req.state.pose.position.z = pose["z"]
        req.state.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
        req.state.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
        req.state.twist = Twist()
        self._set_state_cli.call_async(req)
        self.get_logger().info(
            f"Robot Gazebo'da kat {floor_int} konumuna tasindi "
            f"(x={pose['x']:.2f}, y={pose['y']:.2f}, z={pose['z']:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RotavElevatorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
