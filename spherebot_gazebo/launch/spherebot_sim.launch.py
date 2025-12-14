import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
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

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Whether to start RViz')

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
        arguments=[
            '-topic', 'robot_description',
            '-name', 'spherebot',
            '-x', '-3.69',   # Set X position to 0
            '-y', '0.0',   # Set Y position to 0
            '-z', '0.1',   # Keep Z offset to avoid spawning inside the floor
            '-Y', '0.0'    # Set Yaw (Rotation around Z) to 0.0 (Facing East/X+)
        ],
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

    # Odom Fixer Node
    # Adds frame_ids to /diff_drive_base_controller/odom and publishes to .../odom_fixed
    odom_fixer_node = Node(
        package='spherebot_control',
        executable='odom_fixer',
        name='odom_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory(description_pkg), 'rviz', 'urdf_config.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

# --- INTEGRATION: Sphere Spawner ---
    
    # 1. Bridge specifically for the spawn service
    # This runs alongside your existing bridge without conflict.
    # It bridges the service that allows us to create objects in the 'assessment_world'.
    spawn_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/assessment_world/create@ros_gz_interfaces/srv/SpawnEntity'],
        output='screen'
    )

    # 2. The Spawner Node
    # We call the script directly from the assessment_world package so you don't need to copy it.
    spawn_spheres_node = Node(
        package='assessment_world',
        executable='spawn_spheres.py',
        name='sphere_spawner',
        output='screen'
    )

    # 3. Delay to ensure Gazebo is fully loaded before spawning
    # We wait 5 seconds (just like the tutor's launch file) to ensure the world exists.
    delayed_spawn_spheres = TimerAction(
        period=5.0,
        actions=[spawn_spheres_node]
    )

    return LaunchDescription([
        declare_rviz_cmd,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster,
        diff_drive_spawner,
        left_gate_spawner,
        right_gate_spawner,
        bridge,
        robot_localization_node,
        safety_stop_node,
        odom_fixer_node,
        rviz_node,
        spawn_service_bridge,
        delayed_spawn_spheres
    ])