"""
navigation.launch.py
====================
Launch only the navigation + wormhole stack (no perception):
  - Nav2 bringup
  - wormhole_navigator (Python)
  - multi_floor_navigator (C++)
  - elevator_controller

Useful for testing floor-switching logic with a known map and
without running the heavier detection pipeline.

Usage
-----
  ros2 launch hotel_luggage_robot navigation.launch.py \\
      initial_map:=floor_1 \\
      map_yaml:=/path/to/floor_1.yaml \\
      wormhole_db:=/path/to/hotel_wormholes.db \\
      maps_dir:=/path/to/maps/
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("hotel_luggage_robot")
    nav2_share = get_package_share_directory("nav2_bringup")

    args = [
        DeclareLaunchArgument("use_sim_time",  default_value="false"),
        DeclareLaunchArgument("initial_map",   default_value="floor_1"),
        DeclareLaunchArgument("map_yaml",      default_value=""),
        DeclareLaunchArgument("wormhole_db",   default_value=""),
        DeclareLaunchArgument("maps_dir",      default_value=""),
        DeclareLaunchArgument("nav2_params",
            default_value=os.path.join(pkg_share, "config", "hotel_nav2.yaml")),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_map  = LaunchConfiguration("initial_map")
    map_yaml     = LaunchConfiguration("map_yaml")
    wormhole_db  = LaunchConfiguration("wormhole_db")
    maps_dir     = LaunchConfiguration("maps_dir")
    nav2_params  = LaunchConfiguration("nav2_params")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map":          map_yaml,
            "params_file":  nav2_params,
            "autostart":    "true",
        }.items(),
    )

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

    multi_floor_nav = Node(
        package="hotel_luggage_robot",
        executable="multi_floor_navigator",
        name="multi_floor_navigator",
        output="screen",
        parameters=[
            nav2_params,
            {
                "use_sim_time":     use_sim_time,
                "initial_floor":    initial_map,
                "wormhole_db_path": wormhole_db,
                "maps_base_path":   maps_dir,
            },
        ],
    )

    elevator_ctrl = Node(
        package="hotel_luggage_robot",
        executable="elevator_controller.py",
        name="elevator_controller",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        args + [
            LogInfo(msg="=== Hotel Robot Navigation Stack Starting ==="),
            nav2,
            wormhole_nav,
            multi_floor_nav,
            elevator_ctrl,
        ]
    )
