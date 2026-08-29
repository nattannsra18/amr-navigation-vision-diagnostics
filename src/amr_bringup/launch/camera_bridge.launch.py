from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="camera_image_bridge",
        arguments=[
            "/camera/color/image_raw",
            "/camera/depth/image_raw",
        ],
        parameters=[
            {"qos": "sensor_data"},
        ],
        output="screen",
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=[
            (
                "/camera/color/camera_info"
                "@sensor_msgs/msg/CameraInfo"
                "[gz.msgs.CameraInfo"
            ),
        ],
        parameters=[
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return LaunchDescription([
        image_bridge,
        camera_info_bridge,
    ])
