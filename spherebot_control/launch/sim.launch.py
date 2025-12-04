import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # --- Paths ---
    description_pkg = 'spherebot_description'
    control_pkg = 'spherebot_control'
    
    xacro_file = os.path.join(get_package_share_directory(description_pkg), 'urdf', 'spherebot.urdf.xacro')
    
    # Path to the new controller config in spherebot_control
    controller_config = os.path.join(get_package_share_directory(control_pkg), 'config', 'controllers.yaml')

    # --- Robot Description ---
    # We pass the controller config path to xacro so the URDF knows where to look
    robot_description_content = Command([
        'xacro ', xacro_file, 
        ' config_file:=', controller_config
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # --- Nodes ---

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 2. Gazebo (using ros_gz_sim)
    # We launch the empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Spawn Entity
    # Spawns the robot into Gazebo using the topic published by robot_state_publisher
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'spherebot',
                   '-z', '0.1'], # Spawn slightly above ground
        output='screen'
    )

    # 4. Bridge (ROS <-> Gazebo)
    # This bridges the clock and sensors
    # 4. Bridge (ROS <-> Gazebo)
    # This bridges the clock and sensors
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # 5. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
    )

    left_gate_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gate_controller"],
    )

    right_gate_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gate_controller"],
    )

    # Delay start of spawners until after simulation is up (optional, but good practice)
    # Often standard spawners wait for the controller_manager service automatically.

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        left_gate_spawner,
        right_gate_spawner
    ])