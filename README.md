# Terrain Mapping and Autonomous Drone Control

This project implements a drone control system capable of autonomous spiral trajectory navigation, real-time environment perception, and precision landing using ArUco markers.

## Features Implemented

### Spiral Trajectory Navigation

The drone successfully performs a descending spiral trajectory from an initial height. This trajectory is executed using PX4 offboard control by publishing `TrajectorySetpoint` messages via ROS 2.

* The spiral starts at 20 meters altitude.
* The drone maintains yaw alignment and performs a gradual descent in a circular pattern.
* Gimbal stabilization and orientation tracking are integrated during the flight.

### ArUco Marker Detection *(In Progress / Integrated)*

We developed an ArUco detection node using OpenCV that subscribes to the drone’s camera feed and:

* Detects ArUco markers using the `cv2.aruco` module.
* Estimates the 3D pose of each marker.
* Publishes the pose as a `geometry_msgs/PoseStamped` message to the `/aruco_marker/pose` topic.

This lays the foundation for visual servoing and precision landing.

### Precision Landing Over ArUco Marker *(In Progress / Integrated)*

An autonomous landing node was added to:

* Subscribe to the ArUco marker pose.
* Continuously align the drone above the marker.
* Trigger a landing maneuver once the drone is within a small threshold (10 cm) of the marker’s center.

This node publishes trajectory setpoints for alignment and sends `VehicleCommand` to initiate landing using PX4.

---

## Project Structure

```
terrain_mapping_drone_control/
├── launch/
├── models/
├── config/
├── scripts/
│   ├── save_pointcloud_node.py           # (for later point cloud saving)
├── terrain_mapping_drone_control/
│   ├── spiral_trajectory_controller.py   # spiral trajectory execution
│   ├── aruco_tracker.py                  # ArUco detection + pose publishing
│   ├── cylinder_landing_node.py          # ArUco-based autonomous landing
```

---

## How to Run

### 1. Run PX4 Autopilot + Gazebo

```bash
make px4_sitl gazebo
```

### 2. Launch RViz (optional for visualization)

```bash
rviz2
```

### 3. Run the spiral trajectory node

```bash
python3 src/terrain_mapping_drone_control/terrain_mapping_drone_control/spiral_trajectory_controller.py
```

### 4. Run the ArUco detection node

```bash
python3 src/terrain_mapping_drone_control/terrain_mapping_drone_control/aruco_tracker.py
```

### 5. Run the autonomous landing controller

```bash
python3 src/terrain_mapping_drone_control/terrain_mapping_drone_control/cylinder_landing_node.py
```

---

## Future Scope

* Estimate cylinder height from captured point clouds using Open3D.
* Improve ArUco marker robustness using Kalman filtering.
* Integrate SLAM-based localization for drift correction.
* Enable multi-marker tracking for prioritized landing.

---

## Google Drive Video Link : https://drive.google.com/file/d/11piGprLGRdePsd7EuLd6nS774QlhrQOj/view?usp=sharing

---

**Author:** Dhiren Makwana
**Last updated:** May 11, 2025
