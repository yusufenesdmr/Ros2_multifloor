#!/usr/bin/env python3
"""
person_detector.py  –  Hotel Luggage Robot
===========================================
Real-time person detection via camera (YOLOv8) + LiDAR fusion.

Pipeline
--------
1. Subscribe to camera image → run YOLOv8 → get 2-D bounding boxes
2. Subscribe to LaserScan   → build point cloud in camera frame
3. For each YOLO person bbox, pick the closest LiDAR point inside it
   to get a metric distance estimate
4. Project bounding box centroid + metric depth → map-frame position
   using TF2 transforms
5. Track persons across frames with a simple IoU-based tracker
6. Publish DetectedPersonArray + call nav2_virtual_layer AddCircle to
   create dynamic costmap obstacles around each person
7. Markers published for RViz2 visualisation

Requires
--------
  pip install ultralytics          (YOLOv8)
  pip install opencv-python-headless
  ros2 packages: sensor_msgs, geometry_msgs, visualization_msgs, tf2_ros,
                 cv_bridge, image_geometry
"""

import math
import struct
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Point, Vector3, TransformStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 – registers converters

# Hotel Luggage Robot custom messages
from hotel_luggage_robot.msg import DetectedPerson, DetectedPersonArray

# nav2_virtual_layer services (from repo 1)
try:
    from nav2_virtual_layer.srv import AddCircle, RemoveShape
    _HAS_VIRTUAL_LAYER = True
except ImportError:
    AddCircle = None
    RemoveShape = None
    _HAS_VIRTUAL_LAYER = False


# ─── Tracking data structure ─────────────────────────────────────────────────
@dataclass
class Track:
    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    last_seen: float = field(default_factory=time.monotonic)
    virtual_layer_uuid: str = ""  # UUID of the costmap circle for this person
    frames_alive: int = 0
    confidence: float = 1.0
    distance_m: float = 99.0
    is_close: bool = False


class PersonDetectorNode(Node):
    """ROS2 node: camera + LiDAR → DetectedPersonArray + virtual layer circles."""

    # Cost level for person obstacles in nav2_virtual_layer
    PERSON_COST    = 253   # HIGH (not lethal so robot can plan around, not through)
    PERSON_RADIUS  = 0.8   # metres – dynamic obstacle radius per person
    PERSON_DURATION = 3.0  # seconds – obstacle expires if person not re-detected

    TRACK_TIMEOUT  = 2.0   # seconds until a lost track is deleted
    MAX_TRACKS     = 50

    def __init__(self):
        super().__init__("person_detector")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("yolo_model",        "yolov8n.pt")
        self.declare_parameter("yolo_confidence",    0.45)
        self.declare_parameter("yolo_device",        "cpu")   # "cuda:0" for GPU
        self.declare_parameter("yolo_imgsz",         416)
        self.declare_parameter("max_detection_fps",  5.0)
        self.declare_parameter("debug_publish_fps",  5.0)
        self.declare_parameter("camera_topic",       "/rotav/front_cam/image_raw")
        self.declare_parameter("camera_info_topic",  "/rotav/front_cam/camera_info")
        self.declare_parameter("lidar_topic",        "/rotav/scan")
        self.declare_parameter("map_frame",          "map")
        self.declare_parameter("camera_frame",       "camera_optical")
        self.declare_parameter("robot_frame",        "base_footprint")
        self.declare_parameter("lidar_frame",        "lidar_link")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("person_radius_m",    self.PERSON_RADIUS)
        self.declare_parameter("person_cost",        self.PERSON_COST)
        self.declare_parameter("virtual_layer_ns",   "/global_costmap/virtual_layer")
        self.declare_parameter("synthetic_scan_topic", "/person_obstacle_scan")
        self.declare_parameter("close_person_range_m", 1.6)
        self.declare_parameter("close_bbox_margin_px", 72)
        self.declare_parameter("close_track_timeout_s", 0.8)
        self.declare_parameter("close_scan_width_scale", 1.9)
        self.declare_parameter("close_scan_range_padding_m", 0.25)
        self.declare_parameter("close_match_distance_m", 1.6)

        self.yolo_model_name  = self.get_parameter("yolo_model").value
        self.conf_threshold   = self.get_parameter("yolo_confidence").value
        self.device           = self.get_parameter("yolo_device").value
        self.yolo_imgsz       = int(self.get_parameter("yolo_imgsz").value)
        self.max_detection_fps = max(0.1, float(self.get_parameter("max_detection_fps").value))
        self.debug_publish_fps = max(0.1, float(self.get_parameter("debug_publish_fps").value))
        self.map_frame        = self.get_parameter("map_frame").value
        self.camera_frame     = self.get_parameter("camera_frame").value
        self.robot_frame      = self.get_parameter("robot_frame").value
        self.lidar_frame      = self.get_parameter("lidar_frame").value
        self.publish_debug    = self.get_parameter("publish_debug_image").value
        self.person_radius_m  = float(self.get_parameter("person_radius_m").value)
        self.close_person_range_m = float(self.get_parameter("close_person_range_m").value)
        self.close_bbox_margin_px = int(self.get_parameter("close_bbox_margin_px").value)
        self.close_track_timeout_s = float(self.get_parameter("close_track_timeout_s").value)
        self.close_scan_width_scale = float(self.get_parameter("close_scan_width_scale").value)
        self.close_scan_range_padding_m = float(self.get_parameter("close_scan_range_padding_m").value)
        self.close_match_distance_m = float(self.get_parameter("close_match_distance_m").value)
        vl_ns                 = self.get_parameter("virtual_layer_ns").value

        # ── Load YOLOv8 ─────────────────────────────────────────────────────
        try:
            from ultralytics import YOLO
            self.yolo = YOLO(self.yolo_model_name)
            self.yolo.to(self.device)
            self.get_logger().info(f"YOLOv8 loaded: {self.yolo_model_name} on {self.device}")
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLOv8: {exc}")
            self.yolo = None

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Bridge ──────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Camera intrinsics (filled on first CameraInfo) ───────────────────
        self.cam_info: Optional[CameraInfo] = None
        self.fx = self.fy = self.cx = self.cy = None

        # ── Latest LiDAR scan ────────────────────────────────────────────────
        self.latest_scan: Optional[LaserScan] = None
        self._scan_angles: Optional[np.ndarray] = None
        self._last_detection_time = 0.0
        self._last_debug_publish_time = 0.0

        # ── Tracker state ────────────────────────────────────────────────────
        self.tracks: Dict[int, Track] = {}
        self.next_id = 1
        self._person_point_offsets = np.array([
            (0.0, 0.0),
            ( self.person_radius_m * 0.45,  0.0),
            (-self.person_radius_m * 0.45,  0.0),
            (0.0,  self.person_radius_m * 0.45),
            (0.0, -self.person_radius_m * 0.45),
            ( self.person_radius_m * 0.32,  self.person_radius_m * 0.32),
            ( self.person_radius_m * 0.32, -self.person_radius_m * 0.32),
            (-self.person_radius_m * 0.32,  self.person_radius_m * 0.32),
            (-self.person_radius_m * 0.32, -self.person_radius_m * 0.32),
        ], dtype=np.float32)

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ────────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, self.get_parameter("camera_topic").value,
            self.image_callback, sensor_qos)

        self.cam_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self.cam_info_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, self.get_parameter("lidar_topic").value,
            self.scan_callback, sensor_qos)

        # ── Publishers ───────────────────────────────────────────────────────
        self.persons_pub = self.create_publisher(
            DetectedPersonArray, "/detected_persons", 10)

        self.marker_pub = self.create_publisher(
            MarkerArray, "/person_markers", 10)
        self.person_points_pub = self.create_publisher(
            PointCloud2, "/detected_person_points", 10)
        self.person_scan_pub = self.create_publisher(
            LaserScan, self.get_parameter("synthetic_scan_topic").value, 10)

        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image, "/person_detector/debug_image", sensor_qos)

        # ── Virtual layer service clients (nav2_virtual_layer – Repo 1) ──────
        if _HAS_VIRTUAL_LAYER:
            self.add_circle_client = self.create_client(
                AddCircle, f"{vl_ns}/add_circle")
            self.remove_shape_client = self.create_client(
                RemoveShape, f"{vl_ns}/remove_shape")
        else:
            self.add_circle_client = None
            self.remove_shape_client = None
            self.get_logger().warn(
                "nav2_virtual_layer not found; person obstacles will not be added to the costmap"
            )

        # ── Cleanup timer ────────────────────────────────────────────────────
        self.create_timer(0.5, self.cleanup_stale_tracks)
        self.create_timer(0.1, self.publish_person_scan)

        self.get_logger().info("PersonDetectorNode ready.")

    # ─── Callbacks ──────────────────────────────────────────────────────────
    def cam_info_callback(self, msg: CameraInfo):
        if self.cam_info is None:
            self.cam_info = msg
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f"Camera intrinsics loaded: fx={self.fx:.1f} fy={self.fy:.1f} "
                f"cx={self.cx:.1f} cy={self.cy:.1f}")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self._scan_angles = msg.angle_min + np.arange(len(msg.ranges), dtype=np.float32) * msg.angle_increment

    def image_callback(self, msg: Image):
        if self.yolo is None or self.cam_info is None:
            return

        now_mono = time.monotonic()
        if now_mono - self._last_detection_time < (1.0 / self.max_detection_fps):
            return
        self._last_detection_time = now_mono

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"CvBridge error: {exc}")
            return

        # ── Run YOLO detection ────────────────────────────────────────────
        results = self.yolo(
            frame,
            classes=[0],
            conf=self.conf_threshold,
            imgsz=self.yolo_imgsz,
            max_det=8,
            verbose=False,
        )

        detections = []  # list of (cx_px, cy_px, w_px, h_px, conf)
        boxes = results[0].boxes if results else []
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            bx = int((x1 + x2) / 2)
            by = int((y1 + y2) / 2)
            bw = int(x2 - x1)
            bh = int(y2 - y1)
            detections.append((bx, by, bw, bh, conf))

        # ── Fuse camera detections with LiDAR points ─────────────────────
        persons_map = []
        lidar_hits = self._project_lidar_to_image(msg.header)

        for (bx, by, bw, bh, conf) in detections:
            fused = self._fuse_person_detection(bx, by, bw, bh, lidar_hits, msg.header)
            if fused is not None:
                persons_map.append({
                    "bbox": (bx, by, bw, bh),
                    "conf": conf,
                    "dist": fused["dist"],
                    "map_pos": fused["map_pos"],
                })

        # ── Update tracker ────────────────────────────────────────────────
        self._update_tracks(persons_map)

        # ── Publish ───────────────────────────────────────────────────────
        self._publish_detected_persons(msg.header)
        self._publish_markers()

        # ── Debug image ───────────────────────────────────────────────────
        if self.publish_debug:
            if now_mono - self._last_debug_publish_time >= (1.0 / self.debug_publish_fps):
                self._last_debug_publish_time = now_mono
                debug = frame.copy()
                for d in detections:
                    bx, by, bw, bh, conf = d
                    cv2.rectangle(debug,
                        (bx - bw//2, by - bh//2), (bx + bw//2, by + bh//2),
                        (0, 255, 0), 2)
                    cv2.putText(debug, f"person {conf:.2f}",
                        (bx - bw//2, by - bh//2 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                for px, py, _depth, _map_pos in lidar_hits[::4]:
                    if 0 <= px < frame.shape[1] and 0 <= py < frame.shape[0]:
                        cv2.circle(debug, (int(px), int(py)), 2, (0, 180, 255), -1)
                for tid, tr in self.tracks.items():
                    px = int(self.cx + tr.x * self.fx / max(tr.y, 0.1)) if self.fy else 10
                    cv2.putText(debug, f"ID:{tid}",
                        (px, 20 + tid * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                self.debug_image_pub.publish(
                    self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    def _project_lidar_to_image(self, header: Header) -> List[Tuple[int, int, float, Tuple[float, float]]]:
        scan = self.latest_scan
        if scan is None or self.fx is None or self._scan_angles is None:
            return []

        try:
            lidar_to_camera = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.lidar_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            lidar_to_map = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.lidar_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return []

        cam_rot, cam_trans = self._transform_to_rt(lidar_to_camera)
        map_rot, map_trans = self._transform_to_rt(lidar_to_map)
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        if not np.any(valid):
            return []

        ranges = ranges[valid]
        angles = self._scan_angles[valid]
        lidar_points = np.stack(
            (ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges)),
            axis=1,
        )
        cam_points = lidar_points @ cam_rot.T + cam_trans
        map_points = lidar_points @ map_rot.T + map_trans

        z_cam = cam_points[:, 2]
        keep = z_cam > 0.05
        if not np.any(keep):
            return []

        cam_points = cam_points[keep]
        map_points = map_points[keep]
        x_cam = cam_points[:, 0]
        y_cam = cam_points[:, 1]
        z_cam = cam_points[:, 2]

        px = (self.cx + (x_cam * self.fx / z_cam)).astype(np.int32)
        py = (self.cy + (y_cam * self.fy / z_cam)).astype(np.int32)
        return [
            (int(px_i), int(py_i), float(z_i), (float(mx), float(my)))
            for px_i, py_i, z_i, (mx, my, _mz) in zip(px, py, z_cam, map_points)
        ]

    def _transform_to_rt(self, tf: TransformStamped) -> Tuple[np.ndarray, np.ndarray]:
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float32)
        trans = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=np.float32)
        return rot, trans

    def _fuse_person_detection(
        self,
        bx: int,
        by: int,
        bw: int,
        bh: int,
        lidar_hits: List[Tuple[int, int, float, Tuple[float, float]]],
        header: Header,
    ) -> Optional[Dict[str, object]]:
        mono_dist = self._estimate_distance_monocular(bh)
        near_mode = mono_dist <= self.close_person_range_m
        left = bx - bw // 2
        right = bx + bw // 2
        top = by - bh // 2
        bottom = by + bh // 2
        foot_y = bottom - max(4, bh // 12)
        lower_band_top = by

        inside = [
            hit for hit in lidar_hits
            if left <= hit[0] <= right and lower_band_top <= hit[1] <= bottom
        ]

        if inside:
            inside.sort(
                key=lambda item: (
                    abs(item[0] - bx) + 0.5 * abs(item[1] - foot_y),
                    item[2],
                )
            )
            sample_count = max(1, min(len(inside), 5))
            robust = inside[:sample_count]
            depth = float(np.median([item[2] for item in robust]))
            map_x = float(np.median([item[3][0] for item in robust]))
            map_y = float(np.median([item[3][1] for item in robust]))
            return {"dist": depth, "map_pos": (map_x, map_y), "near": depth <= self.close_person_range_m}

        expanded = []
        if near_mode:
            margin = self.close_bbox_margin_px + max(bw, bh) // 8
            left -= margin
            right += margin
            top = max(0, lower_band_top - margin // 3)
            bottom += margin
            expanded = [
                hit for hit in lidar_hits
                if left <= hit[0] <= right and top <= hit[1] <= bottom
            ]
            if expanded:
                expanded.sort(
                    key=lambda item: (
                        abs(item[0] - bx) + 0.5 * abs(item[1] - foot_y),
                        item[2],
                    )
                )
                sample_count = max(1, min(len(expanded), 7))
                robust = expanded[:sample_count]
                depth = float(np.median([item[2] for item in robust]))
                map_x = float(np.median([item[3][0] for item in robust]))
                map_y = float(np.median([item[3][1] for item in robust]))
                return {"dist": depth, "map_pos": (map_x, map_y), "near": True}

        # Fallback to monocular estimate if no LiDAR points fall inside bbox.
        map_pos = self._project_monocular_to_map(bx, foot_y, mono_dist, header)
        if map_pos is None:
            return None
        return {"dist": mono_dist, "map_pos": map_pos, "near": near_mode}

    def _estimate_distance_monocular(self, bbox_h_px: int) -> float:
        if self.fy is None or bbox_h_px <= 1:
            return 3.0
        assumed_person_height_m = 1.70
        return max(0.6, min(8.0, assumed_person_height_m * self.fy / bbox_h_px))

    def _project_monocular_to_map(
            self, bx_px, by_px, dist_m, header) -> Optional[Tuple[float, float]]:
        if self.fx is None or self.fy is None:
            return None

        x_cam = (bx_px - self.cx) * dist_m / self.fx
        y_cam = (by_px - self.cy) * dist_m / self.fy
        z_cam = dist_m
        pt_cam = PointStamped()
        pt_cam.header.frame_id = self.camera_frame
        pt_cam.header.stamp = header.stamp
        pt_cam.point.x = x_cam
        pt_cam.point.y = y_cam
        pt_cam.point.z = z_cam

        try:
            pt_map = self.tf_buffer.transform(pt_cam, self.map_frame,
                timeout=rclpy.duration.Duration(seconds=0.1))
            return (pt_map.point.x, pt_map.point.y)
        except Exception:
            return None

    # ─── Tracker ─────────────────────────────────────────────────────────────
    def _update_tracks(self, persons: list):
        now = time.monotonic()
        matched_ids = set()

        for p in persons:
            mx, my = p["map_pos"]
            best_id   = None
            best_dist = self.close_match_distance_m if p.get("near") else 1.0

            for tid, tr in self.tracks.items():
                if tid in matched_ids:
                    continue
                pred_x, pred_y = self._track_position(tr, now)
                d = math.hypot(mx - pred_x, my - pred_y)
                if d < best_dist:
                    best_dist = d
                    best_id   = tid

            if best_id is not None:
                tr = self.tracks[best_id]
                dt = now - tr.last_seen
                pred_x, pred_y = self._track_position(tr, now)
                tr.vx = (mx - pred_x) / max(dt, 0.001)
                tr.vy = (my - pred_y) / max(dt, 0.001)
                tr.x  = mx
                tr.y  = my
                tr.last_seen   = now
                tr.frames_alive += 1
                tr.confidence  = p["conf"]
                tr.distance_m  = float(p.get("dist", tr.distance_m))
                tr.is_close    = bool(p.get("near", tr.distance_m <= self.close_person_range_m))
                matched_ids.add(best_id)
                self._update_virtual_obstacle(tr)
            else:
                if len(self.tracks) < self.MAX_TRACKS:
                    new_tr = Track(
                        track_id=self.next_id,
                        x=mx, y=my,
                        last_seen=now,
                        confidence=p["conf"],
                        distance_m=float(p.get("dist", 99.0)),
                        is_close=bool(p.get("near", False)))
                    self.next_id += 1
                    self.tracks[new_tr.track_id] = new_tr
                    matched_ids.add(new_tr.track_id)
                    self._add_virtual_obstacle(new_tr)

    def _track_position(self, tr: Track, now: float) -> Tuple[float, float]:
        age = max(0.0, now - tr.last_seen)
        if tr.is_close and age <= self.close_track_timeout_s:
            horizon = min(age, 0.35)
            return tr.x + tr.vx * horizon, tr.y + tr.vy * horizon
        return tr.x, tr.y

    def cleanup_stale_tracks(self):
        now = time.monotonic()
        stale = []
        for tid, tr in self.tracks.items():
            timeout_s = self.close_track_timeout_s if tr.is_close else self.TRACK_TIMEOUT
            if now - tr.last_seen > timeout_s:
                stale.append(tid)
        for tid in stale:
            tr = self.tracks.pop(tid)
            self._remove_virtual_obstacle(tr)

    # ─── Virtual Layer interaction (nav2_virtual_layer – Repo 1) ────────────
    def _add_virtual_obstacle(self, tr: Track):
        if self.add_circle_client is None or not self.add_circle_client.service_is_ready():
            return
        req = AddCircle.Request()
        req.x          = tr.x
        req.y          = tr.y
        req.radius     = float(self.get_parameter("person_radius_m").value)
        req.cost_level = int(self.get_parameter("person_cost").value)
        req.duration   = self.PERSON_DURATION
        req.frame_id   = self.map_frame
        req.identifier = f"person_{tr.track_id}"

        future = self.add_circle_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_circle_added(f, tr))

    def _on_circle_added(self, future, tr: Track):
        try:
            resp = future.result()
            if resp.success:
                tr.virtual_layer_uuid = resp.uuid
        except Exception as exc:
            self.get_logger().debug(f"AddCircle failed: {exc}")

    def _update_virtual_obstacle(self, tr: Track):
        """Re-add obstacle at updated position (old one will expire in PERSON_DURATION s)."""
        self._add_virtual_obstacle(tr)

    def _remove_virtual_obstacle(self, tr: Track):
        if (
            self.remove_shape_client is None
            or not tr.virtual_layer_uuid
            or not self.remove_shape_client.service_is_ready()
        ):
            return
        req = RemoveShape.Request()
        req.identifier = tr.virtual_layer_uuid
        self.remove_shape_client.call_async(req)

    # ─── Publishers ──────────────────────────────────────────────────────────
    def _publish_detected_persons(self, header: Header):
        arr = DetectedPersonArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.map_frame

        for tr in self.tracks.values():
            px, py = self._track_position(tr, time.monotonic())
            p = DetectedPerson()
            p.header.stamp      = arr.header.stamp
            p.header.frame_id   = self.map_frame
            p.track_id          = tr.track_id
            p.position.x        = px
            p.position.y        = py
            p.position.z        = 0.0
            p.velocity.x        = tr.vx
            p.velocity.y        = tr.vy
            p.confidence        = tr.confidence
            p.detected_by_camera = True
            p.detected_by_lidar  = (self.latest_scan is not None)
            arr.persons.append(p)

        arr.count = len(arr.persons)
        self.persons_pub.publish(arr)
        self.person_points_pub.publish(self._build_person_cloud(arr.header))

    def _build_person_cloud(self, header: Header) -> PointCloud2:
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.height = 1
        samples_per_person = 9
        msg.width = len(self.tracks) * samples_per_person
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        payload = bytearray()
        now = time.monotonic()
        for tr in self.tracks.values():
            px, py = self._track_position(tr, now)
            for dx, dy in self._person_point_offsets:
                payload.extend(struct.pack("fff", float(px + dx), float(py + dy), 0.9))
        msg.data = bytes(payload)
        return msg

    def publish_person_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.lidar_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = 8.0
        count = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
        scan.ranges = [float("inf")] * count

        if not self.tracks:
            self.person_scan_pub.publish(scan)
            return

        try:
            map_to_lidar = self.tf_buffer.lookup_transform(
                self.lidar_frame,
                self.map_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            self.person_scan_pub.publish(scan)
            return

        rot, trans = self._transform_to_rt(map_to_lidar)
        now = time.monotonic()
        for tr in self.tracks.values():
            pred_x, pred_y = self._track_position(tr, now)
            person_map = np.array([pred_x, pred_y, 0.0], dtype=np.float32)
            person_lidar = rot @ person_map + trans
            px = float(person_lidar[0])
            py = float(person_lidar[1])
            distance = math.hypot(px, py)
            if distance < scan.range_min or distance > scan.range_max:
                continue

            center_angle = math.atan2(py, px)
            half_width = math.atan2(self.person_radius_m, max(distance, 0.1))
            if distance <= self.close_person_range_m or tr.is_close:
                half_width *= self.close_scan_width_scale
                half_width = max(half_width, math.radians(18.0))
            start_angle = center_angle - half_width
            end_angle = center_angle + half_width
            start_idx = max(0, int((start_angle - scan.angle_min) / scan.angle_increment))
            end_idx = min(count - 1, int((end_angle - scan.angle_min) / scan.angle_increment))
            obstacle_padding = self.person_radius_m * 0.8
            if distance <= self.close_person_range_m or tr.is_close:
                obstacle_padding += self.close_scan_range_padding_m
            obstacle_range = max(scan.range_min, distance - obstacle_padding)
            for idx in range(start_idx, end_idx + 1):
                if obstacle_range < scan.ranges[idx]:
                    scan.ranges[idx] = obstacle_range

        self.person_scan_pub.publish(scan)

    def _publish_markers(self):
        arr = MarkerArray()
        now_stamp = self.get_clock().now().to_msg()

        # Delete all old markers first
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        now = time.monotonic()
        for tr in self.tracks.values():
            px, py = self._track_position(tr, now)
            # Cylinder representing the person
            cyl = Marker()
            cyl.header.frame_id = self.map_frame
            cyl.header.stamp    = now_stamp
            cyl.ns              = "persons"
            cyl.id              = tr.track_id
            cyl.type            = Marker.CYLINDER
            cyl.action          = Marker.ADD
            cyl.pose.position.x = px
            cyl.pose.position.y = py
            cyl.pose.position.z = 0.9  # mid-height of a person
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 0.5
            cyl.scale.y = 0.5
            cyl.scale.z = 1.8  # ~person height
            cyl.color   = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.7)
            cyl.lifetime.sec = 1  # auto-delete if not refreshed
            arr.markers.append(cyl)

            # Text label with ID and velocity
            txt = Marker()
            txt.header.frame_id = self.map_frame
            txt.header.stamp    = now_stamp
            txt.ns              = "person_labels"
            txt.id              = tr.track_id + 10000
            txt.type            = Marker.TEXT_VIEW_FACING
            txt.action          = Marker.ADD
            txt.pose.position.x = px
            txt.pose.position.y = py
            txt.pose.position.z = 2.2
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.3
            speed = math.hypot(tr.vx, tr.vy)
            txt.text  = f"ID:{tr.track_id}\nspd:{speed:.1f}m/s"
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.lifetime.sec = 1
            arr.markers.append(txt)

            # Safety radius circle
            circ = Marker()
            circ.header.frame_id = self.map_frame
            circ.header.stamp    = now_stamp
            circ.ns              = "person_zones"
            circ.id              = tr.track_id + 20000
            circ.type            = Marker.CYLINDER
            circ.action          = Marker.ADD
            circ.pose.position.x = px
            circ.pose.position.y = py
            circ.pose.position.z = 0.01
            circ.pose.orientation.w = 1.0
            r = float(self.get_parameter("person_radius_m").value)
            circ.scale.x = r * 2
            circ.scale.y = r * 2
            circ.scale.z = 0.05
            circ.color   = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.2)
            circ.lifetime.sec = 1
            arr.markers.append(circ)

        self.marker_pub.publish(arr)


# ─── Entry point ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
