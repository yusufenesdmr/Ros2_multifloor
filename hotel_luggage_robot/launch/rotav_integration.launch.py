"""
Full hotel simulation stack:
Gazebo + robot spawn + Nav2/AMCL + hotel task manager + elevator bridge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("hotel_luggage_robot")
    nav2_share = get_package_share_directory("nav2_bringup")

    xacro = os.path.join(pkg_share, "urdf", "rotav.xacro")
    world = os.path.join(pkg_share, "worlds", "hotel_two_floor.world")
    rviz_cfg = os.path.join(pkg_share, "rviz", "hotel_robot.rviz")
    nav2_params = os.path.join(pkg_share, "config", "hotel_nav2.yaml")
    wormhole_db = os.path.join(pkg_share, "database", "hotel_wormholes.db")
    maps_dir = os.path.join(pkg_share, "maps")
    map_yaml = os.path.join(maps_dir, "floor_1.yaml")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_detector", default_value="false"),
        DeclareLaunchArgument("control_gui", default_value="false"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("rviz")
    use_detector = LaunchConfiguration("use_detector")
    control_gui = LaunchConfiguration("control_gui")

    robot_desc = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro]),
        value_type=str,
    )

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            world,
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient", "--verbose"],
        output="screen",
        condition=IfCondition(gui),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="rotav",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": use_sim_time},
        ],
    )

    joint_state_publisher = Node(
        package="hotel_luggage_robot",
        executable="fake_joint_state_publisher.py",
        namespace="rotav",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_rotav",
                output="screen",
                arguments=[
                    "-topic",
                    "/rotav/robot_description",
                    "-entity",
                    "rotav",
                    "-robot_namespace",
                    "rotav",
                    "-x",
                    "-5.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.05",
                    "-timeout",
                    "30",
                ],
            )
        ],
    )

    cmd_vel_relay = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="hotel_luggage_robot",
                executable="cmd_vel_relay.py",
                name="cmd_vel_relay",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_yaml,
                    "params_file": nav2_params,
                    "autostart": "true",
                    "use_composition": "False",
                    "use_respawn": "False",
                }.items(),
            )
        ],
    )

    initial_pose = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="hotel_luggage_robot",
                executable="publish_initial_pose.py",
                name="publish_initial_pose",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "x": -5.0,
                        "y": 0.0,
                        "yaw": 0.0,
                        "topic": "/initialpose",
                    }
                ],
            )
        ],
    )

    multi_floor_nav = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="hotel_luggage_robot",
                executable="multi_floor_navigator",
                name="multi_floor_navigator",
                output="screen",
                parameters=[
                    nav2_params,
                    {
                        "use_sim_time": use_sim_time,
                        "initial_floor": "floor_1",
                        "wormhole_db_path": wormhole_db,
                        "maps_base_path": maps_dir,
                    },
                ],
            )
        ],
    )

    elevator_bridge = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="hotel_luggage_robot",
                executable="rotav_elevator_bridge.py",
                name="rotav_elevator_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    task_manager = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="hotel_luggage_robot",
                executable="hotel_task_manager.py",
                name="hotel_task_manager",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    person_detector = TimerAction(
        period=11.0,
        actions=[
            Node(
                package="hotel_luggage_robot",
                executable="person_detector.py",
                name="person_detector",
                output="screen",
                parameters=[
                    os.path.join(pkg_share, "config", "detection.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            )
        ],
        condition=IfCondition(use_detector),
    )

    hotel_gui = Node(
        package="hotel_luggage_robot",
        executable="hotel_gui.py",
        name="hotel_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(control_gui),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        args
        + [
            gzserver,
            gzclient,
            joint_state_publisher,
            rsp,
            spawn,
            cmd_vel_relay,
            nav2,
            elevator_bridge,
            multi_floor_nav,
            task_manager,
            initial_pose,
            person_detector,
            hotel_gui,
            rviz,
        ]
    )
