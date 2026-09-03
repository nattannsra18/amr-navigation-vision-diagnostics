from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup = FindPackageShare("nav2_bringup")
    amr_bringup = FindPackageShare("amr_bringup")
    amr_simulation = FindPackageShare("amr_simulation")

    nav2_launch = PathJoinSubstitution([
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

    # Custom TurtleBot3 model containing RGB and depth cameras
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

    set_gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        models_path,
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "world": world_file,
            "robot_sdf": robot_sdf,
            "x_pose": "-2.0",
            "y_pose": "-0.5",
            "z_pose": "0.01",
            "yaw": "0.0",
            "slam": "True",
            "params_file": params_file,
            "use_sim_time": "True",
            "autostart": "True",
            "headless": "False",
        }.items(),
    )

    camera_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_bridge_launch),
    )

    return LaunchDescription([
        set_gz_resource_path,        
        simulation,
        camera_bridge,
    ])
