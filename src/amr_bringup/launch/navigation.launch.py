from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup = FindPackageShare("nav2_bringup")
    amr_bringup = FindPackageShare("amr_bringup")
    amr_simulation = FindPackageShare("amr_simulation")

    tb3_simulation_launch = PathJoinSubstitution([
        nav2_bringup,
        "launch",
        "tb3_simulation_launch.py",
    ])

    params_file = PathJoinSubstitution([
        amr_bringup,
        "config",
        "nav2_params.yaml",
    ])

    world_file = PathJoinSubstitution([
        amr_simulation,
        "worlds",
        "warehouse.sdf",
    ])

    map_file = PathJoinSubstitution([
        amr_simulation,
        "maps",
        "warehouse_map.yaml",
    ])

    robot_sdf = PathJoinSubstitution([
        amr_simulation,
        "models",
        "turtlebot3_waffle_rgbd.sdf.xacro",
    ])

    camera_bridge_launch = PathJoinSubstitution([
        amr_bringup,
        "launch",
        "camera_bridge.launch.py",
    ])

    models_path = PathJoinSubstitution([
        amr_simulation,
        "models",
    ])

    headless = LaunchConfiguration("headless")
    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Gazebo without its GUI",
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        models_path,
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_simulation_launch),
        launch_arguments={
            "world": world_file,
            "map": map_file,
            "robot_sdf": robot_sdf,
            "x_pose": "-2.0",
            "y_pose": "-0.5",
            "z_pose": "0.01",
            "yaw": "0.0",
            "slam": "False",
            "params_file": params_file,
            "use_sim_time": "True",
            "autostart": "True",
            "headless": headless,
        }.items(),
    )

    diagnostics = Node(
        package="amr_diagnostics",
        executable="system_monitor",
        name="amr_system_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }],
    )
    camera_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_bridge_launch),
    )
    aruco_detector = Node(
        package="amr_vision",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }],
    )
    return LaunchDescription([
        declare_headless,
        set_gz_resource_path,
        navigation,
        diagnostics,
        camera_bridge,
        aruco_detector,
    ])
