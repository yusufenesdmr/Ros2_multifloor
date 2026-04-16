#!/usr/bin/env python3
"""
hotel_task_manager.py  –  Hotel Luggage Robot
==============================================
High-level task management node.

Responsibilities
----------------
• Expose a ROS2 action server for LuggageDelivery requests
• Maintain a priority queue of pending deliveries
• Dispatch tasks to the multi_floor_navigator action server
• Monitor virtual layer for person-proximity warnings
• Publish robot state + task list for a hotel dashboard

REST / Hotel PMS Integration (stub)
------------------------------------
The `_poll_pms_tasks()` method can be connected to a hotel Property
Management System API (e.g., Opera, Mews) via HTTP to auto-create
delivery tasks from guest check-in events.

Topic / Service API
-------------------
  Action  /hotel_robot/luggage_delivery   (LuggageDelivery)  – incoming requests
  Topic   /hotel_robot/task_queue         (String, JSON)      – current queue
  Topic   /hotel_robot/state              (String)            – IDLE / BUSY / ERROR
  Topic   /hotel_robot/current_floor      (String)            – from navigator
"""

import json
import queue
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from typing import Optional

import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from hotel_luggage_robot.action import LuggageDelivery
from hotel_luggage_robot.msg import DetectedPersonArray


@dataclass(order=True)
class DeliveryTask:
    priority: int                       # lower = higher priority (for heapq)
    task_id: str = field(compare=False)
    guest_name: str  = field(compare=False)
    guest_room: str  = field(compare=False)
    pickup_floor: str = field(compare=False)
    pickup_x: float  = field(compare=False)
    pickup_y: float  = field(compare=False)
    pickup_yaw: float = field(compare=False, default=0.0)
    dropoff_floor: str = field(compare=False, default="")
    dropoff_x: float  = field(compare=False, default=0.0)
    dropoff_y: float  = field(compare=False, default=0.0)
    dropoff_yaw: float = field(compare=False, default=0.0)
    luggage_count: int = field(compare=False, default=1)
    created_at: float = field(compare=False, default_factory=time.time)
    status: str = field(compare=False, default="QUEUED")
    goal_handle: object = field(compare=False, default=None)
    result_msg: object = field(compare=False, default=None)
    done_event: object = field(compare=False, default=None)


class HotelTaskManager(Node):
    """Orchestrator that queues and dispatches luggage delivery tasks."""

    STATES = ("IDLE", "BUSY", "ERROR", "CHARGING")

    def __init__(self):
        super().__init__("hotel_task_manager")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("max_queue_size",  20)
        self.declare_parameter("dispatch_rate_hz", 1.0)

        self._cb_group = ReentrantCallbackGroup()

        # ── Task queue ────────────────────────────────────────────────────────
        self._task_queue: queue.PriorityQueue = queue.PriorityQueue(
            maxsize=self.get_parameter("max_queue_size").value
        )
        self._active_task: Optional[DeliveryTask] = None
        self._robot_state: str = "IDLE"
        self._current_floor:  str = "floor_1"
        self._persons_nearby: int = 0

        # ── Publishers ────────────────────────────────────────────────────────
        self._state_pub      = self.create_publisher(String, "/hotel_robot/state",       10)
        self._queue_pub      = self.create_publisher(String, "/hotel_robot/task_queue",  10)
        self._announce_pub   = self.create_publisher(String, "/hotel_robot/announcement",10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(String,              "/hotel_robot/current_floor",
                                 self._floor_cb,    10)
        self.create_subscription(DetectedPersonArray, "/detected_persons",
                                 self._persons_cb,  10)

        # ── Action server: accept incoming LuggageDelivery requests ──────────
        self._ld_server = ActionServer(
            self, LuggageDelivery,
            "/hotel_task_manager/request_delivery",
            execute_callback    = self._receive_delivery_request,
            goal_callback       = self._goal_cb,
            cancel_callback     = self._cancel_cb,
            callback_group      = self._cb_group,
        )

        # ── Action client: dispatch to multi_floor_navigator ──────────────────
        self._nav_client = ActionClient(
            self, LuggageDelivery,
            "/hotel_robot/luggage_delivery",
            callback_group=self._cb_group,
        )

        # ── Periodic timers ───────────────────────────────────────────────────
        rate = self.get_parameter("dispatch_rate_hz").value
        self.create_timer(1.0 / rate, self._dispatch_next_task,
                          callback_group=self._cb_group)
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info("HotelTaskManager ready.")

    # ─── Action server callbacks ─────────────────────────────────────────────
    def _goal_cb(self, _goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _receive_delivery_request(self, goal_handle):
        """
        Accept a new delivery request, queue it, and keep the action open until
        the navigator finishes. This lets GUI clients receive real feedback and
        the final result instead of a premature "queued" success.
        """
        goal = goal_handle.request
        task = DeliveryTask(
            priority     = int(2 - goal.priority),   # 0=normal → 2, 2=urgent → 0
            task_id      = goal.task_id or str(uuid.uuid4())[:8],
            guest_name   = goal.guest_name,
            guest_room   = goal.guest_room,
            pickup_floor = goal.pickup_floor,
            pickup_x     = goal.pickup_x,
            pickup_y     = goal.pickup_y,
            pickup_yaw   = goal.pickup_yaw,
            dropoff_floor= goal.dropoff_floor,
            dropoff_x    = goal.dropoff_x,
            dropoff_y    = goal.dropoff_y,
            dropoff_yaw  = goal.dropoff_yaw,
            luggage_count= goal.luggage_count,
            goal_handle  = goal_handle,
            done_event   = threading.Event(),
        )

        try:
            self._task_queue.put_nowait(task)
            self.get_logger().info(
                f"Queued delivery [{task.task_id}]: "
                f"Guest {goal.guest_name} room {goal.guest_room} | "
                f"priority={goal.priority}"
            )
        except queue.Full:
            result = LuggageDelivery.Result()
            result.success = False
            result.message = "Task queue full – try again later"
            goal_handle.abort()
            return result

        queued_feedback = LuggageDelivery.Feedback()
        queued_feedback.phase = "QUEUED"
        queued_feedback.current_floor = self._current_floor
        queued_feedback.progress_percent = 0.0
        goal_handle.publish_feedback(queued_feedback)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                result = LuggageDelivery.Result()
                result.success = False
                result.message = "Task cancelled while waiting in queue"
                goal_handle.canceled()
                return result
            if task.done_event.wait(timeout=0.25):
                break

        result = task.result_msg or LuggageDelivery.Result()
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ─── Dispatcher ──────────────────────────────────────────────────────────
    def _dispatch_next_task(self):
        """Pull the highest-priority task and dispatch it to the navigator."""
        if self._robot_state != "IDLE":
            return
        if self._task_queue.empty():
            return

        task: DeliveryTask = self._task_queue.get_nowait()
        task.status = "DISPATCHING"
        self._active_task = task
        self._set_state("BUSY")

        self.get_logger().info(
            f"Dispatching task [{task.task_id}] for {task.guest_name} "
            f"room {task.guest_room} | "
            f"pickup={task.pickup_floor}({task.pickup_x:.1f},{task.pickup_y:.1f}) "
            f"dropoff={task.dropoff_floor}({task.dropoff_x:.1f},{task.dropoff_y:.1f})"
        )

        # Build LuggageDelivery goal
        nav_goal = LuggageDelivery.Goal()
        nav_goal.task_id       = task.task_id
        nav_goal.guest_name    = task.guest_name
        nav_goal.guest_room    = task.guest_room
        nav_goal.pickup_floor  = task.pickup_floor
        nav_goal.pickup_x      = task.pickup_x
        nav_goal.pickup_y      = task.pickup_y
        nav_goal.pickup_yaw    = task.pickup_yaw
        nav_goal.dropoff_floor = task.dropoff_floor
        nav_goal.dropoff_x     = task.dropoff_x
        nav_goal.dropoff_y     = task.dropoff_y
        nav_goal.dropoff_yaw   = task.dropoff_yaw
        nav_goal.luggage_count = task.luggage_count
        nav_goal.priority      = max(0, 2 - task.priority)  # convert back

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigator action server unavailable")
            self._set_state("ERROR")
            self._active_task = None
            return

        opts = rclpy.action.client.ClientGoalHandle  # type hint
        send_future = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self._nav_feedback_cb,
        )
        send_future.add_done_callback(self._nav_goal_accepted_cb)

    def _nav_goal_accepted_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Navigator rejected the task")
            self._set_state("IDLE")
            self._active_task = None
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        if self._active_task:
            self._active_task.status = fb.phase
            if self._active_task.goal_handle is not None:
                self._active_task.goal_handle.publish_feedback(fb)
        self.get_logger().debug(
            f"[{fb.phase}] floor={fb.current_floor} "
            f"progress={fb.progress_percent:.0f}% "
            f"persons={fb.persons_detected_nearby}"
        )

    def _nav_result_cb(self, future):
        result = future.result().result
        task = self._active_task

        if result.success:
            self.get_logger().info(
                f"Task [{task.task_id if task else '?'}] COMPLETED "
                f"in {result.total_time_sec:.1f}s | "
                f"floors traversed: {result.floors_traversed}"
            )
            self._announce(
                f"Luggage delivered to {task.guest_name} room {task.guest_room}"
                if task else "Delivery completed"
            )
        else:
            self.get_logger().error(
                f"Task [{task.task_id if task else '?'}] FAILED: {result.message}"
            )

        if task is not None:
            task.result_msg = result
            if task.done_event is not None:
                task.done_event.set()

        self._active_task = None
        self._set_state("IDLE")

    # ─── Subscriptions ────────────────────────────────────────────────────────
    def _floor_cb(self, msg: String):
        self._current_floor = msg.data

    def _persons_cb(self, msg: DetectedPersonArray):
        self._persons_nearby = msg.count

    # ─── Helpers ─────────────────────────────────────────────────────────────
    def _set_state(self, state: str):
        self._robot_state = state
        self._state_pub.publish(String(data=state))

    def _announce(self, text: str):
        self._announce_pub.publish(String(data=text))

    def _publish_status(self):
        status = {
            "state":           self._robot_state,
            "current_floor":   self._current_floor,
            "persons_nearby":  self._persons_nearby,
            "queue_size":      self._task_queue.qsize(),
            "active_task":     (
                {
                    "task_id":    self._active_task.task_id,
                    "guest_name": self._active_task.guest_name,
                    "guest_room": self._active_task.guest_room,
                    "status":     self._active_task.status,
                }
                if self._active_task else None
            ),
            "timestamp": time.time(),
        }
        self._state_pub.publish(String(data=self._robot_state))
        self._queue_pub.publish(String(data=json.dumps(status, indent=2)))


def main(args=None):
    rclpy.init(args=args)
    node = HotelTaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
