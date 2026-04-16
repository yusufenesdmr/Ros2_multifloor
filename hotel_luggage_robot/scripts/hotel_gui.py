#!/usr/bin/env python3
"""Tkinter room panel backed by a ROS2 action client."""

import queue
import threading
import tkinter as tk
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from hotel_luggage_robot.action import LuggageDelivery


ROOMS = [
    ("101", "floor_1", -9.5, -2.15, 3.14159),
    ("102", "floor_1", -6.5, -2.15, 3.14159),
    ("103", "floor_1", -3.5, -2.15, 3.14159),
    ("104", "floor_1", -0.5, -2.15, 3.14159),
    ("105", "floor_1", -9.5, 2.15, 0.0),
    ("106", "floor_1", -6.5, 2.15, 0.0),
    ("107", "floor_1", -3.5, 2.15, 0.0),
    ("108", "floor_1", -0.5, 2.15, 0.0),
    ("201", "floor_2", -9.5, -2.15, 3.14159),
    ("202", "floor_2", -6.5, -2.15, 3.14159),
    ("203", "floor_2", -3.5, -2.15, 3.14159),
    ("204", "floor_2", -0.5, -2.15, 3.14159),
    ("205", "floor_2", -9.5, 2.15, 0.0),
    ("206", "floor_2", -6.5, 2.15, 0.0),
    ("207", "floor_2", -3.5, 2.15, 0.0),
    ("208", "floor_2", -0.5, 2.15, 0.0),
]


class HotelGuiNode(Node):
    def __init__(self, status_queue: queue.Queue):
        super().__init__("hotel_gui")
        self._status_queue = status_queue
        self._client = ActionClient(
            self,
            LuggageDelivery,
            "/hotel_task_manager/request_delivery",
        )
        self._current_goal = None
        self._status_queue.put("GUI hazir")

    def send_room_goal(self, room_id: str, floor: str, x: float, y: float, yaw: float):
        if not self._client.wait_for_server(timeout_sec=1.0):
            self._status_queue.put("Task manager bagli degil")
            return

        goal = LuggageDelivery.Goal()
        goal.task_id = f"GUI-{room_id}-{uuid.uuid4().hex[:6]}"
        goal.guest_name = "DirectRoomNav"
        goal.guest_room = room_id
        goal.pickup_floor = floor
        goal.pickup_x = x
        goal.pickup_y = y
        goal.pickup_yaw = yaw
        goal.dropoff_floor = floor
        goal.dropoff_x = x
        goal.dropoff_y = y
        goal.dropoff_yaw = yaw
        goal.luggage_count = 0
        goal.priority = 1

        self._status_queue.put(f"{room_id} icin komut gonderiliyor")
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._status_queue.put("Gorev reddedildi")
            return
        self._current_goal = goal_handle
        self._status_queue.put("Gorev kabul edildi")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        floor = fb.current_floor or "?"
        phase = fb.phase or "?"
        progress = int(fb.progress_percent)
        self._status_queue.put(f"{phase} | {floor} | %{progress}")

    def _result_cb(self, future):
        result = future.result().result
        if result.success:
            self._status_queue.put(f"Tamamlandi | kat={result.final_floor}")
        else:
            self._status_queue.put(f"Hata: {result.message}")


def main():
    status_queue: queue.Queue[str] = queue.Queue()

    rclpy.init()
    node = HotelGuiNode(status_queue)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title("Hotel Robot Panel")
    root.configure(bg="#111827")

    status = tk.StringVar(value="Bir oda sec")

    tk.Label(
        root,
        text="Oda Secimi",
        font=("Helvetica", 18, "bold"),
        bg="#111827",
        fg="#f9fafb",
    ).pack(pady=(18, 8))

    frame = tk.Frame(root, bg="#111827")
    frame.pack(padx=20, pady=10)

    for idx, (room_id, floor, x, y, yaw) in enumerate(ROOMS):
        button = tk.Button(
            frame,
            text=room_id,
            width=8,
            bg="#2563eb" if floor == "floor_1" else "#059669",
            fg="white",
            relief="flat",
            command=lambda r=room_id, f=floor, rx=x, ry=y, ryaw=yaw: node.send_room_goal(r, f, rx, ry, ryaw),
        )
        button.grid(row=idx // 4, column=idx % 4, padx=6, pady=6)

    tk.Label(
        root,
        textvariable=status,
        font=("Helvetica", 11),
        bg="#111827",
        fg="#d1d5db",
    ).pack(pady=(8, 18))

    def poll_status():
        while True:
            try:
                status.set(status_queue.get_nowait())
            except queue.Empty:
                break
        root.after(150, poll_status)

    def on_close():
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(150, poll_status)
    root.mainloop()


if __name__ == "__main__":
    main()
