#!/usr/bin/env python3
"""
Complete Mission Launch File
Launches everything needed for the spherebot mission in one command:
- Gazebo simulation
- Nav2 stack with RViz
- Vision detector
- Mission control
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = 'spherebot_gazebo'
    pkg_nav = 'spherebot_navigation'

    # ========================================================
    # 1. GAZEBO SIMULATION
    # ========================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_gazebo),
                'launch',
                'spherebot_sim.launch.py'
            )
        )
    )

    # ========================================================
    # 2. NAV2 STACK + RVIZ
    # ========================================================
    # Delay to let Gazebo fully start
    navigation_launch = TimerAction(
        period=15.0,  # Wait 3 seconds for Gazebo to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(pkg_nav),
                        'launch',
                        'navigation.launch.py'
                    )
                )
            )
        ]
    )

    # ========================================================
    # 3. VISION DETECTOR NODE
    # ========================================================
    # Delay to let camera topic be available
    vision_detector = TimerAction(
        period=15.0,  # Wait 5 seconds for sensors to be ready
        actions=[
            Node(
                package='spherebot_navigation',
                executable='vision_detector',
                name='vision_detector',
                output='screen'
            )
        ]
    )

    # ========================================================
    # 4. MISSION CONTROL NODE
    # ========================================================
    # Delay to let Nav2 fully initialize
    mission_control = TimerAction(
        period=8.0,  # Wait 8 seconds for Nav2 to be active
        actions=[
            Node(
                package='spherebot_navigation',
                executable='mission_control',
                name='mission_control',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        navigation_launch,
        vision_detector,
        mission_control
    ])
