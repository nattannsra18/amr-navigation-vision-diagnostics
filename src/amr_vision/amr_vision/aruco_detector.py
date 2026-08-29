#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter(
            "image_topic",
            "/camera/color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/color/camera_info",
        )
        self.declare_parameter(
            "marker_length",
            0.45,
        )
        self.declare_parameter(
            "camera_frame",
            "camera_rgb_optical_frame",
        )
        self.declare_parameter(
            "publish_tf",
            True,
        )
        self.declare_parameter(
            "marker_frame_prefix",
            "aruco_marker_",
        )

        image_topic = self.get_parameter(
            "image_topic"
        ).value

        camera_info_topic = self.get_parameter(
            "camera_info_topic"
        ).value

        self.marker_length = float(
            self.get_parameter("marker_length").value
        )

        self.camera_frame = str(
            self.get_parameter("camera_frame").value
        )

        self.publish_tf = bool(
            self.get_parameter("publish_tf").value
        )

        self.marker_frame_prefix = str(
            self.get_parameter(
                "marker_frame_prefix"
            ).value
        )

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_matrix = None
        self.distortion_coefficients = None
        self.last_detected_ids = None

        self.aruco_dictionary = (
            cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50
            )
        )

        self.detector_parameters = (
            cv2.aruco.DetectorParameters_create()
        )

        self.detector_parameters.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX
        )

        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.camera_info_subscription = (
            self.create_subscription(
                CameraInfo,
                camera_info_topic,
                self.camera_info_callback,
                qos_profile_sensor_data,
            )
        )

        self.marker_ids_publisher = self.create_publisher(
            Int32MultiArray,
            "/aruco/marker_ids",
            10,
        )

        self.marker_poses_publisher = self.create_publisher(
            PoseArray,
            "/aruco/poses",
            10,
        )

        self.debug_image_publisher = self.create_publisher(
            Image,
            "/aruco/debug_image",
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Listening for images on {image_topic}"
        )

        self.get_logger().info(
            "Listening for camera information on "
            f"{camera_info_topic}"
        )

        self.get_logger().info(
            "ArUco dictionary: DICT_4X4_50, "
            f"marker length: {self.marker_length:.2f} m"
        )

        self.get_logger().info(
            f"Marker TF broadcasting: {self.publish_tf}, "
            f"frame prefix: {self.marker_frame_prefix}"
        )

    def camera_info_callback(self, message):
        first_message = self.camera_matrix is None

        self.camera_matrix = np.array(
            message.k,
            dtype=np.float64,
        ).reshape((3, 3))

        if message.d:
            self.distortion_coefficients = np.array(
                message.d,
                dtype=np.float64,
            ).reshape((-1, 1))
        else:
            self.distortion_coefficients = np.zeros(
                (5, 1),
                dtype=np.float64,
            )

        if first_message:
            self.get_logger().info(
                "Camera calibration received: "
                f"{message.width}x{message.height}"
            )

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except CvBridgeError as error:
            self.get_logger().error(
                f"Unable to convert camera image: {error}"
            )
            return

        gray_image = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY,
        )

        corners, marker_ids, _ = (
            cv2.aruco.detectMarkers(
                gray_image,
                self.aruco_dictionary,
                parameters=self.detector_parameters,
            )
        )

        debug_image = image.copy()

        marker_ids_message = Int32MultiArray()

        poses_message = PoseArray()
        poses_message.header = message.header

        if not poses_message.header.frame_id:
            poses_message.header.frame_id = (
                self.camera_frame
            )

        detected_ids = []

        if marker_ids is not None and len(marker_ids) > 0:
            detected_ids = [
                int(value)
                for value in marker_ids.flatten()
            ]

            marker_ids_message.data = detected_ids

            cv2.aruco.drawDetectedMarkers(
                debug_image,
                corners,
                marker_ids,
            )

            if self.camera_matrix is not None:
                self.estimate_marker_poses(
                    debug_image,
                    corners,
                    marker_ids,
                    poses_message,
                )

        self.marker_ids_publisher.publish(
            marker_ids_message
        )

        self.marker_poses_publisher.publish(
            poses_message
        )

        self.publish_debug_image(
            debug_image,
            message,
        )

        current_ids = tuple(detected_ids)

        if current_ids != self.last_detected_ids:
            if current_ids:
                self.get_logger().info(
                    "Detected ArUco marker IDs: "
                    f"{list(current_ids)}"
                )

            elif self.last_detected_ids:
                self.get_logger().info(
                    "No ArUco marker currently visible"
                )

            self.last_detected_ids = current_ids

    def estimate_marker_poses(
        self,
        debug_image,
        corners,
        marker_ids,
        poses_message,
    ):
        try:
            rvecs, tvecs, _ = (
                cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_length,
                    self.camera_matrix,
                    self.distortion_coefficients,
                )
            )

        except cv2.error as error:
            self.get_logger().warning(
                f"Pose estimation failed: {error}"
            )
            return

        for index, marker_id in enumerate(
            marker_ids.flatten()
        ):
            rotation_vector = (
                rvecs[index].reshape(3)
            )

            translation_vector = (
                tvecs[index].reshape(3)
            )

            cv2.drawFrameAxes(
                debug_image,
                self.camera_matrix,
                self.distortion_coefficients,
                rotation_vector,
                translation_vector,
                self.marker_length * 0.5,
            )

            rotation_matrix, _ = cv2.Rodrigues(
                rotation_vector
            )

            quaternion = (
                self.rotation_matrix_to_quaternion(
                    rotation_matrix
                )
            )

            pose = Pose()

            pose.position.x = float(
                translation_vector[0]
            )

            pose.position.y = float(
                translation_vector[1]
            )

            pose.position.z = float(
                translation_vector[2]
            )

            pose.orientation.x = float(
                quaternion[0]
            )

            pose.orientation.y = float(
                quaternion[1]
            )

            pose.orientation.z = float(
                quaternion[2]
            )

            pose.orientation.w = float(
                quaternion[3]
            )

            poses_message.poses.append(pose)

            if self.publish_tf:
                self.publish_marker_transform(
                    int(marker_id),
                    pose,
                    poses_message.header,
                )

    def publish_marker_transform(
        self,
        marker_id,
        pose,
        source_header,
    ):
        transform = TransformStamped()

        transform.header.stamp = (
            source_header.stamp
        )

        transform.header.frame_id = (
            source_header.frame_id
            or self.camera_frame
        )

        transform.child_frame_id = (
            f"{self.marker_frame_prefix}{marker_id}"
        )

        transform.transform.translation.x = (
            pose.position.x
        )

        transform.transform.translation.y = (
            pose.position.y
        )

        transform.transform.translation.z = (
            pose.position.z
        )

        transform.transform.rotation.x = (
            pose.orientation.x
        )

        transform.transform.rotation.y = (
            pose.orientation.y
        )

        transform.transform.rotation.z = (
            pose.orientation.z
        )

        transform.transform.rotation.w = (
            pose.orientation.w
        )

        self.tf_broadcaster.sendTransform(
            transform
        )

    def publish_debug_image(
        self,
        debug_image,
        source_message,
    ):
        try:
            debug_message = (
                self.bridge.cv2_to_imgmsg(
                    debug_image,
                    encoding="bgr8",
                )
            )

        except CvBridgeError as error:
            self.get_logger().error(
                f"Unable to create debug image: {error}"
            )
            return

        debug_message.header = (
            source_message.header
        )

        if not debug_message.header.frame_id:
            debug_message.header.frame_id = (
                self.camera_frame
            )

        self.debug_image_publisher.publish(
            debug_message
        )

    @staticmethod
    def rotation_matrix_to_quaternion(matrix):
        trace = np.trace(matrix)

        if trace > 0.0:
            scale = math.sqrt(
                trace + 1.0
            ) * 2.0

            qw = 0.25 * scale

            qx = (
                matrix[2, 1] - matrix[1, 2]
            ) / scale

            qy = (
                matrix[0, 2] - matrix[2, 0]
            ) / scale

            qz = (
                matrix[1, 0] - matrix[0, 1]
            ) / scale

        elif (
            matrix[0, 0] > matrix[1, 1]
            and matrix[0, 0] > matrix[2, 2]
        ):
            scale = math.sqrt(
                1.0
                + matrix[0, 0]
                - matrix[1, 1]
                - matrix[2, 2]
            ) * 2.0

            qw = (
                matrix[2, 1] - matrix[1, 2]
            ) / scale

            qx = 0.25 * scale

            qy = (
                matrix[0, 1] + matrix[1, 0]
            ) / scale

            qz = (
                matrix[0, 2] + matrix[2, 0]
            ) / scale

        elif matrix[1, 1] > matrix[2, 2]:
            scale = math.sqrt(
                1.0
                + matrix[1, 1]
                - matrix[0, 0]
                - matrix[2, 2]
            ) * 2.0

            qw = (
                matrix[0, 2] - matrix[2, 0]
            ) / scale

            qx = (
                matrix[0, 1] + matrix[1, 0]
            ) / scale

            qy = 0.25 * scale

            qz = (
                matrix[1, 2] + matrix[2, 1]
            ) / scale

        else:
            scale = math.sqrt(
                1.0
                + matrix[2, 2]
                - matrix[0, 0]
                - matrix[1, 1]
            ) * 2.0

            qw = (
                matrix[1, 0] - matrix[0, 1]
            ) / scale

            qx = (
                matrix[0, 2] + matrix[2, 0]
            ) / scale

            qy = (
                matrix[1, 2] + matrix[2, 1]
            ) / scale

            qz = 0.25 * scale

        quaternion = np.array(
            [qx, qy, qz, qw],
            dtype=np.float64,
        )

        quaternion_norm = np.linalg.norm(
            quaternion
        )

        if quaternion_norm == 0.0:
            return np.array(
                [0.0, 0.0, 0.0, 1.0],
                dtype=np.float64,
            )

        return quaternion / quaternion_norm


def main(args=None):
    rclpy.init(args=args)

    node = ArucoDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
