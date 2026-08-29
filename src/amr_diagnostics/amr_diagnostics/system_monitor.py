#!/usr/bin/env python3

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32MultiArray


class AmrSystemMonitor(Node):

    def __init__(self):
        super().__init__('amr_system_monitor')

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10,
        )

        # Sensor timestamps
        self.last_scan_time = None
        self.last_odom_time = None
        self.last_cmd_vel_time = None
        self.last_camera_time = None
        self.last_marker_ids_time = None
        self.last_marker_pose_time = None

        # LiDAR state
        self.minimum_scan_range = None

        # Odometry state
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Velocity command state
        self.commanded_linear_velocity = 0.0
        self.commanded_angular_velocity = 0.0

        # Camera state
        self.camera_width = 0
        self.camera_height = 0
        self.camera_encoding = 'unknown'
        self.camera_frame_id = ''

        # ArUco state
        self.marker_ids = []
        self.marker_pose_frame = ''
        self.marker_position = None
        self.marker_distance = None

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Int32MultiArray,
            '/aruco/marker_ids',
            self.marker_ids_callback,
            10,
        )

        self.create_subscription(
            PoseArray,
            '/aruco/poses',
            self.marker_poses_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_diagnostics,
        )

        self.log_counter = 0

        self.get_logger().info('AMR system monitor started')

    def current_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def scan_callback(self, message):
        self.last_scan_time = self.current_time()

        valid_ranges = [
            distance
            for distance in message.ranges
            if (
                math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            )
        ]

        if valid_ranges:
            self.minimum_scan_range = min(valid_ranges)
        else:
            self.minimum_scan_range = None

    def odom_callback(self, message):
        self.last_odom_time = self.current_time()

        self.linear_velocity = message.twist.twist.linear.x
        self.angular_velocity = message.twist.twist.angular.z

    def cmd_vel_callback(self, message):
        self.last_cmd_vel_time = self.current_time()

        self.commanded_linear_velocity = message.linear.x
        self.commanded_angular_velocity = message.angular.z

    def camera_callback(self, message):
        self.last_camera_time = self.current_time()

        self.camera_width = message.width
        self.camera_height = message.height
        self.camera_encoding = message.encoding
        self.camera_frame_id = message.header.frame_id

    def marker_ids_callback(self, message):
        self.last_marker_ids_time = self.current_time()

        new_marker_ids = [
            int(marker_id)
            for marker_id in message.data
        ]

        if not new_marker_ids or new_marker_ids != self.marker_ids:
            self.marker_position = None
            self.marker_distance = None

        self.marker_ids = new_marker_ids

    def marker_poses_callback(self, message):
        self.last_marker_pose_time = self.current_time()
        self.marker_pose_frame = message.header.frame_id

        if not message.poses:
            self.marker_position = None
            self.marker_distance = None
            return

        position = message.poses[0].position

        self.marker_position = (
            position.x,
            position.y,
            position.z,
        )

        self.marker_distance = math.sqrt(
            position.x ** 2
            + position.y ** 2
            + position.z ** 2
        )

    def make_status(self, name, level, message, values):
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = 'turtlebot3_waffle_sim'
        status.level = level
        status.message = message

        status.values = [
            KeyValue(
                key=key,
                value=str(value),
            )
            for key, value in values.items()
        ]

        return status

    def lidar_status(self, now):
        if self.last_scan_time is None:
            return self.make_status(
                'AMR/LiDAR',
                DiagnosticStatus.ERROR,
                'No LaserScan data received',
                {
                    'topic': '/scan',
                },
            )

        age = now - self.last_scan_time

        if age > 2.0:
            return self.make_status(
                'AMR/LiDAR',
                DiagnosticStatus.ERROR,
                'LaserScan data timeout',
                {
                    'topic': '/scan',
                    'age_seconds': f'{age:.2f}',
                },
            )

        if self.minimum_scan_range is None:
            return self.make_status(
                'AMR/LiDAR',
                DiagnosticStatus.WARN,
                'No valid range measurements',
                {
                    'topic': '/scan',
                },
            )

        if self.minimum_scan_range < 0.20:
            return self.make_status(
                'AMR/LiDAR',
                DiagnosticStatus.WARN,
                'Obstacle very close',
                {
                    'minimum_range_m': (
                        f'{self.minimum_scan_range:.3f}'
                    ),
                },
            )

        return self.make_status(
            'AMR/LiDAR',
            DiagnosticStatus.OK,
            'LaserScan healthy',
            {
                'minimum_range_m': (
                    f'{self.minimum_scan_range:.3f}'
                ),
                'age_seconds': f'{age:.2f}',
            },
        )

    def odometry_status(self, now):
        if self.last_odom_time is None:
            return self.make_status(
                'AMR/Odometry',
                DiagnosticStatus.ERROR,
                'No odometry data received',
                {
                    'topic': '/odom',
                },
            )

        age = now - self.last_odom_time

        if age > 2.0:
            return self.make_status(
                'AMR/Odometry',
                DiagnosticStatus.ERROR,
                'Odometry data timeout',
                {
                    'topic': '/odom',
                    'age_seconds': f'{age:.2f}',
                },
            )

        return self.make_status(
            'AMR/Odometry',
            DiagnosticStatus.OK,
            'Odometry healthy',
            {
                'linear_velocity_mps': (
                    f'{self.linear_velocity:.3f}'
                ),
                'angular_velocity_radps': (
                    f'{self.angular_velocity:.3f}'
                ),
                'age_seconds': f'{age:.2f}',
            },
        )

    def command_status(self, now):
        if self.last_cmd_vel_time is None:
            return self.make_status(
                'AMR/VelocityCommand',
                DiagnosticStatus.OK,
                'Robot idle',
                {
                    'topic': '/cmd_vel',
                },
            )

        age = now - self.last_cmd_vel_time

        if age > 2.0:
            return self.make_status(
                'AMR/VelocityCommand',
                DiagnosticStatus.OK,
                'No recent velocity command',
                {
                    'last_command_age_seconds': f'{age:.2f}',
                },
            )

        return self.make_status(
            'AMR/VelocityCommand',
            DiagnosticStatus.OK,
            'Velocity command active',
            {
                'linear_x': (
                    f'{self.commanded_linear_velocity:.3f}'
                ),
                'angular_z': (
                    f'{self.commanded_angular_velocity:.3f}'
                ),
            },
        )

    def camera_status(self, now):
        if self.last_camera_time is None:
            return self.make_status(
                'AMR/Camera',
                DiagnosticStatus.ERROR,
                'No RGB camera data received',
                {
                    'topic': '/camera/color/image_raw',
                },
            )

        age = now - self.last_camera_time

        if age > 2.0:
            return self.make_status(
                'AMR/Camera',
                DiagnosticStatus.ERROR,
                'RGB camera data timeout',
                {
                    'topic': '/camera/color/image_raw',
                    'age_seconds': f'{age:.2f}',
                },
            )

        return self.make_status(
            'AMR/Camera',
            DiagnosticStatus.OK,
            'RGB camera healthy',
            {
                'topic': '/camera/color/image_raw',
                'resolution': (
                    f'{self.camera_width}x{self.camera_height}'
                ),
                'encoding': self.camera_encoding,
                'frame_id': self.camera_frame_id,
                'age_seconds': f'{age:.2f}',
            },
        )

    def aruco_status(self, now):
        if self.last_marker_ids_time is None:
            return self.make_status(
                'AMR/ArUco',
                DiagnosticStatus.ERROR,
                'No ArUco detector data received',
                {
                    'topic': '/aruco/marker_ids',
                },
            )

        marker_age = now - self.last_marker_ids_time

        if marker_age > 2.0:
            return self.make_status(
                'AMR/ArUco',
                DiagnosticStatus.ERROR,
                'ArUco detector data timeout',
                {
                    'topic': '/aruco/marker_ids',
                    'age_seconds': f'{marker_age:.2f}',
                },
            )

        if not self.marker_ids:
            return self.make_status(
                'AMR/ArUco',
                DiagnosticStatus.WARN,
                'No ArUco marker detected',
                {
                    'topic': '/aruco/marker_ids',
                    'detector_age_seconds': f'{marker_age:.2f}',
                },
            )

        if self.last_marker_pose_time is None:
            return self.make_status(
                'AMR/ArUco',
                DiagnosticStatus.WARN,
                'Marker detected but pose unavailable',
                {
                    'marker_ids': str(self.marker_ids),
                },
            )

        pose_age = now - self.last_marker_pose_time

        if pose_age > 2.0 or self.marker_position is None:
            return self.make_status(
                'AMR/ArUco',
                DiagnosticStatus.WARN,
                'Marker pose data unavailable',
                {
                    'marker_ids': str(self.marker_ids),
                    'pose_age_seconds': f'{pose_age:.2f}',
                },
            )

        x, y, z = self.marker_position

        return self.make_status(
            'AMR/ArUco',
            DiagnosticStatus.OK,
            'ArUco marker detected',
            {
                'marker_ids': str(self.marker_ids),
                'first_marker_id': self.marker_ids[0],
                'distance_m': f'{self.marker_distance:.3f}',
                'position_x_m': f'{x:.3f}',
                'position_y_m': f'{y:.3f}',
                'position_z_m': f'{z:.3f}',
                'frame_id': self.marker_pose_frame,
                'detector_age_seconds': f'{marker_age:.2f}',
                'pose_age_seconds': f'{pose_age:.2f}',
            },
        )

    def publish_diagnostics(self):
        now = self.current_time()

        statuses = [
            self.lidar_status(now),
            self.odometry_status(now),
            self.command_status(now),
            self.camera_status(now),
            self.aruco_status(now),
        ]

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = statuses

        self.diagnostics_publisher.publish(message)

        self.log_counter += 1

        if self.log_counter % 5 == 0:
            summary = ', '.join(
                (
                    f"{status.name.split('/')[-1]}"
                    f'={status.message}'
                )
                for status in statuses
            )

            self.get_logger().info(summary)


def main(args=None):
    rclpy.init(args=args)
    node = AmrSystemMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
