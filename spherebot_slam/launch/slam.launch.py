import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam = 'spherebot_slam'
    
    # Path to our config file
    slam_config = os.path.join(get_package_share_directory(pkg_slam), 'config', 'mapper_params_online_async.yaml')

    # 1. SLAM Toolbox
    # We use the standard launch file provided by slam_toolbox
    start_async_slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={'slam_params_file': slam_config, 'use_sim_time': 'true'}.items()
    )

    # 2. RViz
    # We launch RViz to visualize the map building
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # We can add a default config later, for now, we open empty
        # arguments=['-d', os.path.join(get_package_share_directory(pkg_slam), 'rviz', 'slam.rviz')] 
    )

    return LaunchDescription([
        start_async_slam_toolbox_node,
        rviz
    ])