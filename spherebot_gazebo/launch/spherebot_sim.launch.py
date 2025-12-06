import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- Paths ---
    description_pkg = 'spherebot_description'
    control_pkg = 'spherebot_control'
    gazebo_pkg = 'spherebot_gazebo'
    ekf_config_path = os.path.join(get_package_share_directory('spherebot_control'), 'config', 'ekf.yaml')
    
    # Path to the Xacro file
    xacro_file = os.path.join(get_package_share_directory(description_pkg), 'urdf', 'spherebot.urdf.xacro')
    
    # Path to the controller config
    controller_config = os.path.join(get_package_share_directory(control_pkg), 'config', 'controllers.yaml')

    # --- NEW: Path to the World File ---
    # This matches the folder structure we defined in setup.py ('worlds/')
    world_path = os.path.join(get_package_share_directory(gazebo_pkg), 'worlds', 'assessment_world.sdf')

    # --- Robot Description ---
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

    # 2. Gazebo Simulation
    # We pass the full path to the world file in gz_args
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': '-r ' + world_path}.items(),
    )

    # 3. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'spherebot',
                   '-z', '0.1'],
        output='screen'
    )

    # 4. Controller Spawners
    joint_state_broadcaster = Node(
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

    # 5. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory(gazebo_pkg), 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # Safety Stop Node
    # Intercepts /cmd_vel (from teleop) and publishes to /diff_drive_base_controller/cmd_vel_unstamped
    safety_stop_node = Node(
        package='spherebot_control',
        executable='safety_stop',
        name='safety_stop',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster,
        diff_drive_spawner,
        left_gate_spawner,
        right_gate_spawner,
        bridge,
        robot_localization_node,
        safety_stop_node
    ])