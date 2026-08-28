from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_simulation_launch),
        launch_arguments={
            "world": world_file,
            "map": map_file,
            "slam": "False",
            "params_file": params_file,
            "use_sim_time": "True",
            "autostart": "True",
            "headless": "False",
        }.items(),
    )

    return LaunchDescription([
        navigation,
    ])
