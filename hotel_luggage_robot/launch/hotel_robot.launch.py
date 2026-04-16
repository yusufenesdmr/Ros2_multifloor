"""
hotel_robot.launch.py  –  Main launch file
==========================================
Starts the complete hotel luggage robot system:
  1. Nav2 stack (map_server, amcl, controller, planner, bt_navigator …)
  2. Multi-floor navigator (C++ action server)
  3. Wormhole navigator (Python, ROS2 Humble rclpy)
  4. Person detector (camera + LiDAR, YOLOv8)
  5. Elevator controller (simulation or real hardware bridge)
  6. Hotel task manager (high-level delivery orchestration)

Usage
-----
  ros2 launch hotel_luggage_robot hotel_robot.launch.py \\
      initial_map:=floor_1 \\
      map_yaml:=/path/to/floor_1.yaml \\
      wormhole_db:=/path/to/hotel_wormholes.db \\
      maps_dir:=/path/to/maps/ \\
      use_sim_time:=false
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("hotel_luggage_robot")
    nav2_share = get_package_share_directory("nav2_bringup")

    # ── Launch arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("use_sim_time",   default_value="false"),
        DeclareLaunchArgument("initial_map",    default_value="floor_1",
            description="Name of the floor the robot starts on"),
        DeclareLaunchArgument("map_yaml",       default_value="",
            description="Path to the initial floor YAML map file"),
        DeclareLaunchArgument("wormhole_db",    default_value="",
            description="Path to the SQLite wormhole database"),
        DeclareLaunchArgument("maps_dir",       default_value="",
            description="Directory containing SDF2MAP-generated map YAMLs"),
        DeclareLaunchArgument("nav2_params",
            default_value=os.path.join(pkg_share, "config", "hotel_nav2.yaml"),
            description="Nav2 parameters YAML file"),
        DeclareLaunchArgument("rviz",           default_value="true",
            description="Launch RViz2 for visualisation"),
        DeclareLaunchArgument("log_level",      default_value="info"),
    ]

    use_sim_time  = LaunchConfiguration("use_sim_time")
    initial_map   = LaunchConfiguration("initial_map")
    map_yaml      = LaunchConfiguration("map_yaml")
    wormhole_db   = LaunchConfiguration("wormhole_db")
    maps_dir      = LaunchConfiguration("maps_dir")
    nav2_params   = LaunchConfiguration("nav2_params")

    # ── Nav2 bringup ─────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time":    use_sim_time,
            "map":             map_yaml,
            "params_file":     nav2_params,
            "autostart":       "true",
        }.items(),
    )

    # ── Multi-floor navigator (C++) ───────────────────────────────────────────
    multi_floor_nav = Node(
        package="hotel_luggage_robot",
        executable="multi_floor_navigator",
        name="multi_floor_navigator",
        output="screen",
        parameters=[
            nav2_params,
            {
                "use_sim_time":    use_sim_time,
                "initial_floor":   initial_map,
                "wormhole_db_path": wormhole_db,
                "maps_base_path":  maps_dir,
            },
        ],
    )

    # ── Wormhole navigator (Python, ROS2 Humble) ──────────────────────────────
    wormhole_nav = Node(
        package="hotel_luggage_robot",
        executable="wormhole_navigator.py",
        name="wormhole_navigator",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "wormhole_navigator.yaml"),
            {
                "use_sim_time":     use_sim_time,
                "initial_map":      initial_map,
                "wormhole_db_path": wormhole_db,
                "maps_base_dir":    maps_dir,
            },
        ],
    )

    # ── Person detector (camera + LiDAR + YOLOv8) ────────────────────────────
    person_detector = Node(
        package="hotel_luggage_robot",
        executable="person_detector.py",
        name="person_detector",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "detection.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── Elevator controller ───────────────────────────────────────────────────
    elevator_ctrl = Node(
        package="hotel_luggage_robot",
        executable="elevator_controller.py",
        name="elevator_controller",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── Hotel task manager ────────────────────────────────────────────────────
    task_manager = Node(
        package="hotel_luggage_robot",
        executable="hotel_task_manager.py",
        name="hotel_task_manager",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz", "hotel_robot.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        args + [
            LogInfo(msg="=== Hotel Luggage Robot Starting ==="),
            nav2_launch,
            multi_floor_nav,
            wormhole_nav,
            person_detector,
            elevator_ctrl,
            task_manager,
            rviz_node,
        ]
    )
