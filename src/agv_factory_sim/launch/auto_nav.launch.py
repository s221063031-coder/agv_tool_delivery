import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory('agv_factory_sim')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    # Path to your generated map
    map_file = os.path.join(package_dir, 'maps', 'factory_map.yaml')

    # 1. Start Gazebo and Robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'gazebo.launch.py'))
    )

    # 2. Start Nav2 with Static Map (A* Planner)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([gazebo, nav2])
