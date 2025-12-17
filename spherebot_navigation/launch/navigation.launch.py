import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = 'spherebot_navigation'
    pkg_nav2_bringup = 'nav2_bringup'

    # --- Paths ---
    map_file = os.path.join(get_package_share_directory(pkg_nav), 'maps', 'assessment_map.yaml')
    params_file = os.path.join(get_package_share_directory(pkg_nav), 'config', 'nav2_params.yaml')



    # --- Launch Nav2 Bringup ---
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_nav2_bringup), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    # --- RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory(pkg_nav2_bringup), 'rviz', 'nav2_default_view.rviz')]
    )

    return LaunchDescription([
        nav2_bringup,
        rviz
    ])