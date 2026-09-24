# AMR base driver

The ESP32 firmware publishes calibrated raw MPU6050 data and x2 wheel counts.

The deployed ODROID-C4 link uses its 3.3 V `UART_EE_A` on the 40-pin header:
pin 8 TX to ESP32 GPIO3 RX0, pin 10 RX from ESP32 GPIO1 TX0, and pin 6 to ESP32
ground. The ROS bridge opens `/dev/ttyAML6` at 115200 baud. ESP32 power is
supplied separately. Disconnect ODROID pins 8 and 10 whenever the ESP32 USB
cable is connected for flashing because UART0 is shared with the DevKit USB
bridge.
The ROS bridge publishes `/imu/data_raw`, `/wheel/odometry`, `/joint_states`,
and `/diagnostics`, and sends left/right RPM targets from `/cmd_vel`.

The USB link uses a fixed 44-byte binary frame with CRC-16. The ESP32 build
uses a 40 MHz flash clock because repeated bootloader checksum failures were
observed at 80 MHz on the physical controller.

Motor output is disabled by default. Before a floor test, measure and update
`wheel_radius` and `wheel_separation`, then launch with
`enable_motors:=true`. The ESP32 command watchdog stops PWM after 300 ms.

The MPU6050 has no magnetometer. The firmware removes startup gyro bias and
rejects bad frames; `robot_localization` fuses wheel forward velocity and yaw
rate from both wheel odometry and the gyro. Long-term absolute yaw still needs
LiDAR localization or another absolute heading source.

Run manually on the ODROID:

```bash
source /opt/ros/jazzy/setup.bash
source ~/amr_ws/install/setup.bash
ros2 launch amr_base_driver base_hardware.launch.py enable_motors:=false
```

Before changing `enable_motors` to `true`, measure the loaded wheel radius and
wheel separation, support the robot so its wheels cannot propel it, verify the
physical motor cutoff, and repeat the PI tests with the robot's real load.

## Measured physical geometry

The frame origin is `base_footprint`, on the floor below the midpoint of the
wheel axle. The axle was measured 0.301-0.305 m from the front; the midpoint
0.303 m is used. `base_link` is one wheel radius (0.0325 m) above it.

| Item | Measured value | ROS transform from `base_link` |
|---|---:|---:|
| Wheel radius | 0.0325 m | n/a |
| Wheel separation | 0.355 m centre-to-centre | n/a |
| LiDAR | x=0.345 m from front, 0.155 m from left body edge, z=0.395 m from floor | x=-0.042, y=-0.005, z=0.3625, yaw=0 |
| MPU6050 | x=0.335 m from front, 0.084 m from left body edge, z=0.075 m from floor | x=-0.032, y=0.066, z=0.0425, yaw=pi |

The conservative rectangular navigation footprint, including the wheels and
rear projection, is `[[0.303, 0.190], [0.303, -0.190], [-0.087, -0.190],
[-0.087, 0.190]]` metres in `base_footprint`. Apply this only to the physical
Nav2 profile; the shared simulation footprint must remain unchanged.

If another launch file already publishes `base_link -> laser_frame`, start
this launch with `publish_sensor_tf:=false` until the duplicate authority is
removed. Exactly one node may publish each TF edge.

## Mapping

The ESP32 and X3 have identical CP2102 serial identifiers, so the ODROID udev
rule assigns stable names from their physical USB topology: `/dev/robot-esp32`
on port 1.4 and `/dev/robot-lidar` on port 1.1. Keep the devices in those USB
sockets or update `99-amr-serial.rules` after moving a cable.

Validate sensors with the wheels raised and motor output locked:

```bash
ros2 launch amr_base_driver mapping.launch.py enable_motors:=false
```

For an attended floor mapping run only, after checking the fuse and reachable
motor cutoff and calibrating loaded geometry:

```bash
ros2 launch amr_base_driver mapping.launch.py enable_motors:=true
```

Save the map from another terminal after completing the route:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/indoor_map
```

## Keyboard floor station

`robot_station.sh station` opens three terminals on the operator computer:
the motor-enabled mapping stack on the ODROID, a watchdog-friendly WASD
controller, and a real-time sensor dashboard. The controller publishes at
20 Hz while a motion key remains fresh and automatically commands zero after
0.35 seconds without another key event. Space or X stops immediately; Q exits.
Press R while stopped to reset wheel odometry and accumulated dashboard
distance before a tape-measure run.

The dashboard shows derived x2 encoder counts, wheel RPM, wheel odometry,
accumulated path length, raw IMU values, LiDAR rate/range, and ESP32 fault
state. Distance remains provisional until the wheel radius is measured under
the robot's real floor load.

Run only with a clear floor area and the physical motor cutoff in reach:

```bash
./robot_station.sh station
```

Send a software stop from another terminal with:

```bash
./robot_station.sh stop
```
