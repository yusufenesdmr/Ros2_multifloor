# Hotel Luggage Robot – Setup Guide

## Prerequisites

- ROS 2 Humble (Ubuntu 22.04)
- Nav2 (`ros-humble-nav2-bringup`)
- nav2-virtual-layer built from https://github.com/sherif1152/nav2-virtual-layer
- SDF2MAP from https://github.com/sherif1152/SDF2MAP (for generating floor maps)
- Python ≥ 3.10
- SQLite3

---

## 1. Build the workspace

```bash
cd ~/ros2_ws/src
git clone <this-repo>           # hotel_luggage_robot
git clone https://github.com/sherif1152/nav2-virtual-layer

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 2. Generate floor maps with SDF2MAP

For each floor of your hotel:

1. Open SDF2MAP
2. Load the floor's `.sdf` or `.world` file
3. Set: Resolution=0.05, LIDAR Height=0.20m, occupied_thresh=0.65
4. Save to `hotel_luggage_robot/maps/floor_N.pgm` + `floor_N.yaml`

Alternatively use any existing maps – just ensure they follow the
YAML format in `maps/README.md`.

---

## 3. Create the wormhole database

```bash
cd ~/ros2_ws/src/hotel_luggage_robot/database/

python3 create_demo_db.py \
    --maps-dir ~/ros2_ws/src/hotel_luggage_robot/maps/ \
    --out hotel_wormholes.db
```

Edit the elevator approach coordinates in `create_demo_db.py` to match
the actual elevator positions in your floor maps before running.

---

## 4. Install Python dependencies

```bash
pip install -r ~/ros2_ws/src/hotel_luggage_robot/requirements.txt
```

---

## 5. Launch

### Full system (navigation + detection + task manager)

```bash
ros2 launch hotel_luggage_robot hotel_robot.launch.py \
    initial_map:=floor_1 \
    map_yaml:=$(pwd)/maps/floor_1.yaml \
    wormhole_db:=$(pwd)/database/hotel_wormholes.db \
    maps_dir:=$(pwd)/maps/
```

### Navigation only (no camera/detection)

```bash
ros2 launch hotel_luggage_robot navigation.launch.py \
    initial_map:=floor_1 \
    map_yaml:=$(pwd)/maps/floor_1.yaml \
    wormhole_db:=$(pwd)/database/hotel_wormholes.db \
    maps_dir:=$(pwd)/maps/
```

### Detection only (test person detector)

```bash
ros2 launch hotel_luggage_robot detection.launch.py \
    camera_topic:=/camera/image_raw \
    lidar_topic:=/scan
```

---

## 6. Send a delivery task

```bash
ros2 action send_goal /hotel_task_manager/request_delivery \
    hotel_luggage_robot/action/LuggageDelivery \
    "{
      task_id: 'TASK001',
      guest_name: 'Ahmet Yilmaz',
      guest_room: '305',
      pickup_floor: 'floor_1',
      pickup_x: 2.0,
      pickup_y: 3.0,
      pickup_yaw: 0.0,
      dropoff_floor: 'floor_3',
      dropoff_x: 10.5,
      dropoff_y: -4.2,
      dropoff_yaw: 1.57,
      luggage_count: 2,
      priority: 1
    }"
```

---

## 7. Monitor the robot

```bash
# Current floor
ros2 topic echo /hotel_robot/current_floor

# Detected persons
ros2 topic echo /detected_persons

# Robot state + task queue (JSON)
ros2 topic echo /hotel_robot/task_queue

# Elevator door state
ros2 topic echo /elevator_controller/door_state
```

---

## Architecture diagram

```
Guest Request
     │
     ▼
HotelTaskManager (/hotel_task_manager/request_delivery)
     │  priority queue
     ▼
WormholeNavigator (/wormhole_navigator/navigate_to_goal)
     │                                    │
     │  AddLine (door wall)               │  LoadMap (floor switch)
     ▼                                    ▼
nav2_virtual_layer              Nav2 map_server
     │                                    │
     │  RemoveShape (door open)           │  AMCL re-init (/initialpose)
     ▼                                    ▼
Nav2 NavigateToPose ──────────────────────┘
     ▲
     │  dynamic person obstacles
PersonDetector
  (YOLOv8 + LiDAR)
     │
     └── AddCircle → nav2_virtual_layer (person obstacles)

ElevatorController
  /elevator_controller/door_state  ──► WormholeNavigator
    "open" = physical confirmation before map switch
```

---

## Key topics / services summary

| Name | Type | Description |
|------|------|-------------|
| `/hotel_robot/luggage_delivery` | Action | C++ multi-floor navigator |
| `/wormhole_navigator/navigate_to_goal` | Action | Python wormhole navigator |
| `/hotel_task_manager/request_delivery` | Action | High-level delivery request |
| `/detected_persons` | Topic | DetectedPersonArray |
| `/person_markers` | Topic | RViz2 visualization |
| `/hotel_robot/current_floor` | Topic | Active floor name |
| `/hotel_robot/state` | Topic | IDLE/BUSY/ERROR |
| `/elevator_controller/call_elevator` | Service | CallElevator |
| `/elevator_controller/door_state` | Topic | open/closed/in_transit |
| `/global_costmap/virtual_layer/add_circle` | Service | Person obstacle |
| `/global_costmap/virtual_layer/add_line` | Service | Door wall |
| `/global_costmap/virtual_layer/remove_shape` | Service | Remove obstacle |
| `/map_server/load_map` | Service | Hot-swap floor map |
| `/initialpose` | Topic | AMCL re-localization |
