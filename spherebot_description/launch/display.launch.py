import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue # <--- Added import

def generate_launch_description():
    pkg_name = 'spherebot_description'
    file_subpath = 'urdf/spherebot.urdf.xacro'

    # Get the path to the Xacro file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    # Use xacro to process the file
    # We wrap the command in ParameterValue to ensure it's treated as a string
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Node 1: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 
                     'use_sim_time': False}]
    )

    # Node 2: Joint State Publisher GUI
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node 3: RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])