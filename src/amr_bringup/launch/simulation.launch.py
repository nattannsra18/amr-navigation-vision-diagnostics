from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    nav2_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "tb3_simulation_launch.py",
            ])
        ),
        launch_arguments={
            "headless": "False",
            "autostart": "True",
            "use_sim_time": "True",
            "params_file": params_file,
        }.items(),
    )

    return LaunchDescription([
        nav2_simulation,
    ])
