"""
detection.launch.py
===================
Launch only the perception stack:
  - person_detector node (YOLOv8 + LiDAR fusion)

Useful for testing detection independently from navigation.

Usage
-----
  ros2 launch hotel_luggage_robot detection.launch.py \\
      camera_topic:=/rotav/front_cam/image_raw \\
      lidar_topic:=/rotav/scan \\
      use_sim_time:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("hotel_luggage_robot")

    args = [
        DeclareLaunchArgument("use_sim_time",        default_value="false"),
        DeclareLaunchArgument("camera_topic",         default_value="/rotav/front_cam/image_raw"),
        DeclareLaunchArgument("camera_info_topic",    default_value="/rotav/front_cam/camera_info"),
        DeclareLaunchArgument("lidar_topic",          default_value="/rotav/scan"),
        DeclareLaunchArgument("yolo_model",           default_value="yolov8n.pt"),
        DeclareLaunchArgument("yolo_confidence",      default_value="0.45"),
        DeclareLaunchArgument("yolo_device",          default_value="cpu"),
        DeclareLaunchArgument("publish_debug_image",  default_value="true"),
        DeclareLaunchArgument("person_radius_m",      default_value="0.8"),
        DeclareLaunchArgument("virtual_layer_ns",
            default_value="/global_costmap/virtual_layer"),
    ]

    person_detector = Node(
        package="hotel_luggage_robot",
        executable="person_detector.py",
        name="person_detector",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "detection.yaml"),
            {
                "use_sim_time":        LaunchConfiguration("use_sim_time"),
                "camera_topic":        LaunchConfiguration("camera_topic"),
                "camera_info_topic":   LaunchConfiguration("camera_info_topic"),
                "lidar_topic":         LaunchConfiguration("lidar_topic"),
                "yolo_model":          LaunchConfiguration("yolo_model"),
                "yolo_confidence":     LaunchConfiguration("yolo_confidence"),
                "yolo_device":         LaunchConfiguration("yolo_device"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                "person_radius_m":     LaunchConfiguration("person_radius_m"),
                "virtual_layer_ns":    LaunchConfiguration("virtual_layer_ns"),
            },
        ],
    )

    return LaunchDescription(args + [person_detector])
