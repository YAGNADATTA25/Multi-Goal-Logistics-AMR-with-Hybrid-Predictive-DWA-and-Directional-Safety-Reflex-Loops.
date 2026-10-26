"""Launch the mixed pedestrian/cart scene using the existing navigation setup."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('autonomous_logistics_sim')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'warehouse_navigation.launch.py')
            ),
            launch_arguments={
                'world': os.path.join(package_dir, 'worlds', 'warehouse_mixed.world'),
            }.items(),
        ),
    ])
