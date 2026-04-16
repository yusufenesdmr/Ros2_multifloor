cmake_minimum_required(VERSION 3.8)
project(hotel_luggage_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# ─── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# SQLite3
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

# ─── Custom messages / services / actions ────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedPerson.msg"
  "msg/DetectedPersonArray.msg"
  "srv/CallElevator.srv"
  "action/LuggageDelivery.action"
  DEPENDENCIES
    geometry_msgs
    std_msgs
    sensor_msgs
)

# Export interfaces so other packages can use them
ament_export_dependencies(rosidl_default_runtime)

# ─── Multi-floor Navigator node (C++) ────────────────────────────────────────
set(NAV_DEPS
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(multi_floor_navigator src/multi_floor_navigator.cpp)
ament_target_dependencies(multi_floor_navigator ${NAV_DEPS})
target_include_directories(multi_floor_navigator PRIVATE
  ${SQLITE3_INCLUDE_DIRS}
)
target_link_libraries(multi_floor_navigator
  yaml-cpp
  ${SQLITE3_LIBRARIES}
)

# Wait for interface generation before building the navigator
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(multi_floor_navigator "${cpp_typesupport_target}")

# ─── Install ─────────────────────────────────────────────────────────────────
install(TARGETS
  multi_floor_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/person_detector.py
  scripts/hotel_task_manager.py
  scripts/elevator_controller.py
  scripts/wormhole_navigator.py
  scripts/rotav_elevator_bridge.py
  scripts/cmd_vel_relay.py
  scripts/hotel_world_generator.py
  scripts/hotel_map_generator.py
  scripts/hotel_gui.py
  scripts/publish_initial_pose.py
  scripts/fake_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  maps
  worlds
  urdf
  rviz
  database
  DESTINATION share/${PROJECT_NAME}
)

# ─── Tests ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
