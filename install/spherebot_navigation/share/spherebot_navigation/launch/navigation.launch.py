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

    # [UPDATED] Fake Localization (Static TF)
    # We apply the -3.69 offset here. 
    # Logic: map_frame = odom_frame + (-3.69 offset)
    # Args: x y z yaw pitch roll parent_frame child_frame
    fake_localization = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-3.69', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

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
        fake_localization,
        nav2_bringup,
        rviz
    ])