# A single person detected by camera + LiDAR fusion
std_msgs/Header header

# Unique tracking ID for this person (persistent across frames)
uint32 track_id

# 2D position in the robot's map frame
geometry_msgs/Point position

# Estimated velocity (m/s) – computed from consecutive detections
geometry_msgs/Vector3 velocity

# Detection confidence [0.0 – 1.0]
float32 confidence

# Source sensors that contributed to this detection
bool detected_by_camera
bool detected_by_lidar

# Bounding box in image coordinates (pixels)
uint32 bbox_x
uint32 bbox_y
uint32 bbox_w
uint32 bbox_h

# Distance from the robot (meters)
float32 distance

# Which floor this person is on
string floor_id
