#!/usr/bin/env python3
"""
elevator_controller.py  –  Hotel Luggage Robot
================================================
Simulated elevator controller node.

In a real deployment this node would interface with the building's BMS
(Building Management System) via MQTT, Modbus, or a hotel API.

Topics/Services exposed
-----------------------
  Service  /elevator_controller/call_elevator  (CallElevator)
  Topic    /elevator_controller/status          (String, JSON)
  Topic    /elevator_controller/door_state      (String)

The node simulates:
  • Elevator travel time proportional to floor distance
  • Door open/close cycle
  • Busy state when elevator is in transit
"""

import json
import math
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hotel_luggage_robot.srv import CallElevator


class ElevatorControllerNode(Node):
    """Simulated elevator interface node."""

    FLOOR_HEIGHTS = {   # metres per floor for travel time calculation
        "floor_1": 0,
        "floor_2": 3,
        "floor_3": 6,
        "floor_4": 9,
        "floor_5": 12,
    }
    TRAVEL_SPEED  = 1.0   # m/s  (realistic hotel elevator ~1-2 m/s)
    DOOR_TIME_SEC = 3.0   # door open/close delay

    def __init__(self):
        super().__init__("elevator_controller")

        self.declare_parameter("num_elevators", 2)

        self._lock    = threading.Lock()
        self._busy    = False
        self._current_floor = "floor_1"

        self._srv = self.create_service(
            CallElevator,
            "/elevator_controller/call_elevator",
            self._handle_call,
        )

        self._status_pub = self.create_publisher(String, "/elevator_controller/status", 10)
        self._door_pub   = self.create_publisher(String, "/elevator_controller/door_state", 10)

        self.create_timer(2.0, self._publish_status)

        self.get_logger().info("ElevatorController (simulation) ready.")

    def _handle_call(self, request, response):
        from_floor = request.current_floor
        to_floor   = request.target_floor

        if self._busy:
            response.success       = False
            response.message       = "Elevator busy – try again"
            response.wait_time_sec = 10.0
            response.elevator_id   = "none"
            return response

        if to_floor not in self.FLOOR_HEIGHTS:
            response.success = False
            response.message = f"Unknown floor: {to_floor}"
            return response

        # Estimate travel time
        h_from = self.FLOOR_HEIGHTS.get(from_floor, 0)
        h_to   = self.FLOOR_HEIGHTS.get(to_floor, 0)
        travel_time = abs(h_to - h_from) / self.TRAVEL_SPEED + self.DOOR_TIME_SEC * 2

        self._busy = True
        # Run elevator simulation in background thread
        t = threading.Thread(
            target=self._simulate_travel,
            args=(from_floor, to_floor, travel_time),
            daemon=True,
        )
        t.start()

        response.success       = True
        response.elevator_id   = "elevator_A"
        response.wait_time_sec = float(travel_time)
        response.message       = (
            f"Elevator dispatched: {from_floor} → {to_floor} "
            f"(ETA {travel_time:.1f}s)"
        )
        self.get_logger().info(response.message)
        return response

    def _simulate_travel(self, from_floor, to_floor, travel_time):
        self.get_logger().info(f"Elevator doors opening at {from_floor}...")
        self._door_pub.publish(String(data="open"))
        time.sleep(self.DOOR_TIME_SEC)

        self._door_pub.publish(String(data="closed"))
        self.get_logger().info(f"Elevator in transit → {to_floor}")
        time.sleep(travel_time - self.DOOR_TIME_SEC * 2)

        self._current_floor = to_floor
        self.get_logger().info(f"Elevator arrived at {to_floor}")
        self._door_pub.publish(String(data="open"))
        time.sleep(self.DOOR_TIME_SEC)

        self._door_pub.publish(String(data="closed"))
        self._busy = False

    def _publish_status(self):
        status = {
            "current_floor": self._current_floor,
            "busy":          self._busy,
            "timestamp":     time.time(),
        }
        self._status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
