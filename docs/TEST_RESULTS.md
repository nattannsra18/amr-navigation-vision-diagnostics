# Complete Test Results

This document records the functional and integration tests performed on the AMR Navigation, Vision & Diagnostics project.

## Test Environment

| Item | Configuration |
|---|---|
| Test date | 2026-08-29 |
| Operating system | Ubuntu 24.04 LTS |
| ROS distribution | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Visualization | RViz2 |
| Robot | Simulated TurtleBot3 Waffle with custom RGB-D model |
| World | Custom warehouse |
| Navigation | Nav2 with AMCL and a saved occupancy map |
| Vision | OpenCV ArUco `DICT_4X4_50`, marker length `0.45 m` |

All commands were run after sourcing the ROS and workspace environments:

```bash
source /opt/ros/jazzy/setup.bash
source ~/amr-navigation-vision-diagnostics/install/setup.bash
```

## 1. Nav2 Lifecycle Test

### Objective

Verify that every managed Nav2 component is configured and active before accepting navigation goals.

### Command

```bash
for node in \
  amcl \
  map_server \
  planner_server \
  controller_server \
  bt_navigator \
  behavior_server \
  smoother_server \
  waypoint_follower \
  velocity_smoother \
  collision_monitor \
  route_server
do
  printf "%-25s " "/$node"
  ros2 lifecycle get "/$node"
done
```

### Result

| Node | State |
|---|---|
| `/amcl` | `active [3]` |
| `/map_server` | `active [3]` |
| `/planner_server` | `active [3]` |
| `/controller_server` | `active [3]` |
| `/bt_navigator` | `active [3]` |
| `/behavior_server` | `active [3]` |
| `/smoother_server` | `active [3]` |
| `/waypoint_follower` | `active [3]` |
| `/velocity_smoother` | `active [3]` |
| `/collision_monitor` | `active [3]` |
| `/route_server` | `active [3]` |

**Acceptance criterion:** every listed node reports `active [3]`.

**Result: PASS**

## 2. LiDAR Test

### Commands

```bash
timeout 5 ros2 topic hz /scan
ros2 topic echo /scan --once --field header
```

### Result

- Measured rate: approximately `5.000 Hz`
- Observed period: approximately `0.200 s`
- Frame ID: `base_scan`
- Publisher: `/ros_gz_bridge`
- Subscribers included AMCL, RViz2, collision monitor, local costmap, and AMR diagnostics.

**Acceptance criterion:** `/scan` continuously publishes finite LaserScan messages near the configured 5 Hz rate, with a TF-connected frame.

**Result: PASS**

## 3. Wheel Odometry Test

### Command

```bash
timeout 5 ros2 topic hz /odom
```

### Result

- Measured rate: approximately `29.4 Hz`
- Observed period: approximately `0.034 s`
- Odometry messages were received continuously by Nav2 and the diagnostic monitor.

**Acceptance criterion:** `/odom` publishes continuously and participates in the `odom → base_footprint` transform chain.

**Result: PASS**

## 4. RGB Camera Test

### Commands

```bash
timeout 5 ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once
```

### Result

- Image rate: approximately `15.2 Hz`
- Resolution: `640 × 480`
- Encoding: `rgb8`
- Image frame: `camera_rgb_optical_frame`
- Camera calibration was available on `/camera/color/camera_info`.
- Representative focal lengths: `fx = fy = 554.3827 px`
- Principal point: `cx = 320.0 px`, `cy = 240.0 px`

**Acceptance criterion:** RGB images and matching camera calibration data are continuously available to the ArUco detector.

**Result: PASS**

## 5. Depth Camera Test

### Commands

```bash
timeout 5 ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/image_raw --once --field header
```

### Result

- Measured rate: approximately `5.001 Hz`
- Observed period: approximately `0.200 s`
- Frame ID: `camera_depth_frame`
- The depth image was displayed successfully in `rqt_image_view`.

**Acceptance criterion:** depth images publish continuously at the configured rate with the correct frame.

**Result: PASS**

### Corrective action found during testing

The depth topic initially produced no ROS messages because `/camera/depth/image_raw` was assigned to the GPU LiDAR sensor in addition to the LiDAR scan topic. The model was corrected so that:

- The GPU LiDAR publishes only `$(arg namespace)/scan`.
- The depth camera publishes `/camera/depth/image_raw`.

After rebuilding and restarting the simulation, both `/scan` and `/camera/depth/image_raw` published at 5 Hz.

## 6. AMCL Localization Test

### Commands

```bash
ros2 topic echo /amcl_pose --once
timeout 5 ros2 run tf2_ros tf2_echo map odom
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
```

### Result

- `/amcl_pose` was received with frame ID `map`.
- The `map → odom` transform became available after the initial pose was set.
- The full `map → odom → base_footprint` chain was available.
- A representative AMCL pose after localization was:
  - Position: `x=-0.078 m`, `y=-0.248 m`
  - Orientation: `z=-0.686`, `w=0.728`
- The TF pose changed while the robot moved, confirming live localization updates. Representative `map → base_footprint` positions progressed from approximately `(0.416, -2.658) m` to `(1.068, -3.060) m` during motion.

**Acceptance criterion:** AMCL publishes a map-frame pose and connects the map frame to the robot base through TF.

**Result: PASS**

### Note about the first TF warning

`tf2_echo` may briefly print `Invalid frame ID "map"` while its listener is discovering the TF tree. In this test, valid transforms followed immediately, so the initial message did not indicate a continuing TF failure.

## 7. Camera TF Test

### Command

```bash
timeout 5 ros2 run tf2_ros tf2_echo map camera_rgb_optical_frame
```

### Result

The camera was connected to the map frame. A representative transform was:

- Translation: `[-0.400, 0.026, 0.117] m`
- RPY: approximately `[-89.954°, 0.000°, -91.722°]`

**Acceptance criterion:** the RGB optical frame is reachable from `map`, enabling camera-frame detections to be transformed into map coordinates.

**Result: PASS**

## 8. Autonomous Navigation Test

### Procedure

1. Launch `navigation.launch.py`.
2. Set the robot pose with RViz2 **2D Pose Estimate**.
3. Wait for AMCL localization to converge.
4. Send a free-space goal with **Nav2 Goal**.
5. Observe `/cmd_vel`, robot motion, local/global plans, and TF updates.

### Command

```bash
timeout 8 ros2 topic hz /cmd_vel
```

### Result

- `/cmd_vel` published at approximately `20 Hz` while the navigation pipeline was active.
- The robot moved toward the selected goal.
- `map → base_footprint` changed continuously while the robot moved.
- LiDAR, local costmap, planner, controller, velocity smoother, and collision monitor remained active.

**Acceptance criterion:** Nav2 accepts the goal, produces velocity commands, moves the robot, and updates its localized pose.

**Result: PASS**

## 9. ArUco Detection Test

### Commands

```bash
ros2 topic echo /aruco/marker_ids --once
ros2 topic echo /aruco/poses --once
```

### Result

- Detected marker ID: `0`
- Pose frame: `camera_rgb_optical_frame`
- Representative marker position in the camera frame:
  - `x=-0.578 m`
  - `y=-0.210 m`
  - `z=3.960 m`
- Calculated distance: approximately `4.008 m`
- The annotated marker boundary, marker ID, and pose axes were visible on `/aruco/debug_image`.

**Acceptance criterion:** a visible `DICT_4X4_50` marker produces an ID, a finite 3D pose, and an annotated debug image.

**Result: PASS**

## 10. ArUco TF Test

### Command

```bash
timeout 5 ros2 run tf2_ros tf2_echo map aruco_marker_0
```

### Result

The marker TF was connected to the map. A representative map-frame transform was:

- Translation: `[8.311, 0.074, 0.330] m`
- Quaternion `(x, y, z, w)`: approximately `[0.006, -0.703, 0.007, 0.711]`

**Acceptance criterion:** a detected marker receives an `aruco_marker_<id>` frame that is reachable from `map`.

**Result: PASS**

## 11. AMR Diagnostics Test

### Command

```bash
ros2 topic echo /diagnostics --once \
  --filter "any(s.name.startswith('AMR/') for s in m.status)"
```

### Result

| Diagnostic | Level | Message | Representative values |
|---|---:|---|---|
| `AMR/LiDAR` | `0` | `LaserScan healthy` | Minimum range `0.551 m`, age `0.20 s` |
| `AMR/Odometry` | `0` | `Odometry healthy` | Linear `0.000 m/s`, angular `0.000 rad/s`, age `0.01 s` |
| `AMR/VelocityCommand` | `0` | `No recent velocity command` | Last command age `187.21 s` while idle |
| `AMR/Camera` | `0` | `RGB camera healthy` | 640×480, `rgb8`, age `0.06 s` |
| `AMR/ArUco` | `0` | `ArUco marker detected` | ID `[0]`, distance `4.008 m`, detector and pose age `0.06 s` |

Level `0` is `DiagnosticStatus.OK`; `ros2 topic echo` may display it as `"\0"`.

**Acceptance criterion:** every expected `AMR/*` entry is present, uses current data, and reports `OK` during a healthy run.

**Result: PASS**

## 12. End-to-End Acceptance Summary

| Subsystem | Result |
|---|---|
| Gazebo warehouse and robot spawn | PASS |
| Simulation clock | PASS |
| LiDAR | PASS |
| Wheel odometry | PASS |
| RGB camera and calibration | PASS |
| Depth camera | PASS |
| Nav2 lifecycle activation | PASS |
| AMCL localization | PASS |
| Robot and sensor TF tree | PASS |
| Autonomous navigation goal | PASS |
| ArUco ID and pose estimation | PASS |
| ArUco map-frame TF | PASS |
| AMR health diagnostics | PASS |

The tested simulation pipeline satisfies the current project scope: saved-map autonomous navigation, RGB-D sensing, ArUco perception, TF integration, and live AMR diagnostics all operate together successfully.
