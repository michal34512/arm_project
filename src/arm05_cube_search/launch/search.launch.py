from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('arm05_cube_search'))
    config = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/search.yaml')),
        *[
            Node(
                package='arm05_cube_search',
                executable=name,
                parameters=[config, {'use_sim_time': True}],
                output='screen',
            )
            for name in ('aruco_detector', 'cube_mapper', 'search_manager')
        ],
    ])