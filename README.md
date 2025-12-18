# SphereBOT - Autonomous Sphere Collection Robot

## Project Overview

SphereBOT is an autonomous mobile robot designed to navigate an assessment environment, detect and collect three colored spheres of different sizes, and transport them to a designated pen area. The robot combines autonomous navigation using ROS2 Nav2 stack with computer vision-based object detection and manual teleoperation for precision tasks.

## Video [link](https://youtu.be/Z_MHkmz-nNo)

## Images
![SphereBot in assessment world](https://github.com/FaseehFramework/src/blob/master/images/spherebot%20in%20assessment%20world.png?raw=true)
![SphereBot costmap](https://github.com/FaseehFramework/src/blob/master/images/costmap.png?raw=true)
![Nav2 goal to Pen Goal](https://github.com/FaseehFramework/src/blob/master/images/pen%20goal.png?raw=true)
![Final goal](https://github.com/FaseehFramework/src/blob/master/images/final%20goal.png?raw=true)



**Mission Objective**: Locate and collect three spheres (blue, green, and red) of varying sizes and return them to a pen enclosure within the assessment world.

**Robot Platform**: Custom differential drive robot with:
- **Sensors**: 2D Lidar, RGB Camera, IMU
- **Actuators**: Differential drive base, dual gate mechanism for sphere capture
- **Navigation**: Nav2 autonomous navigation with AMCL localization
- **Vision**: OpenCV-based color detection for blue sphere tracking

---

## Package Architecture

### `spherebot_description`

**Purpose**: Contains the complete robot model definition in URDF/Xacro format.

**Key Components**:
- **URDF Files**: Robot structure, joints, links, and sensors
  - `spherebot.urdf.xacro`: Parametric robot definition with configurable properties
  - `spherebot.urdf`: Generated robot description
- **Visual & Collision Geometry**: 3D meshes and collision models
- **Sensor Definitions**: 
  - Lidar sensor (360° scanning)
  - RGB camera (front-facing)
  - IMU (inertial measurement unit)
- **RViz Configuration**: Default visualization settings for robot debugging

**Dependencies**: `ros2_control`, `ros2_controllers`, `gz_ros2_control`

#### Launch Files

**`display.launch.py`**
```bash
ros2 launch spherebot_description display.launch.py
```
- Launches RViz with the SphereBOT robot model
- Useful for visualizing and debugging robot URDF
- Shows robot structure, joints, and sensor positions
- Does not require Gazebo simulation

---

### `spherebot_control`

**Purpose**: Manages robot control interfaces, controller configurations, and sensor data processing.

**Key Components**:

#### Configuration Files
- **`controllers.yaml`**: Defines ros2_control controllers
  - `diff_drive_base_controller`: Differential drive base velocity control
  - `left_gate_controller` / `right_gate_controller`: Position controllers for capture gates
  - `joint_state_broadcaster`: Publishes joint states
- **`ekf.yaml`**: Extended Kalman Filter configuration for sensor fusion
  - Fuses odometry, IMU, and visual odometry data
  - Provides stable localization estimates

#### Python Nodes

**`odom_fixer.py`**
- **Function**: Corrects odometry frame IDs for Nav2 compatibility
- **Input**: `/diff_drive_base_controller/odom`
- **Output**: `/diff_drive_base_controller/odom_fixed`
- **Purpose**: Ensures proper frame naming (`odom` → `base_footprint`) required by Nav2

#### Launch Files

**`sim.launch.py`**
```bash
ros2 launch spherebot_control sim.launch.py
```
- Launches SphereBOT in Gazebo simulation environment
- Alternative launcher to `spherebot_gazebo` package
- Initializes controllers and robot state publisher
- Useful for testing control configurations

---

### `spherebot_gazebo`

**Purpose**: Simulation environment setup and Gazebo-ROS integration.

**Key Components**:

#### Launch Files
**`spherebot_sim.launch.py`**
- Primary simulation launcher
- **Functionality**:
  1. Loads assessment world with obstacles and pen area
  2. Spawns SphereBOT at starting position (-3.0, 0.0)
  3. Initializes ros2_control controllers
  4. Spawns three random spheres (different colors/sizes)
  5. Launches ROS-Gazebo bridge for sensor/actuator communication
  6. Starts robot localization (EKF) node
  7. Optionally launches RViz for visualization

#### Configuration
- **`bridge.yaml`**: ROS-Gazebo topic mappings
  - Camera, Lidar, IMU sensor bridges
  - Joint states and command interfaces

#### World Files
- **`assessment_world.sdf`**: Simulation environment
  - Room boundaries and obstacles
  - Pen area for sphere collection
  - Lighting and physics settings

---

### `spherebot_navigation`

**Purpose**: Autonomous navigation, mission planning, and vision-based sphere detection.

**Key Components**:

#### Configuration
- **`nav2_params.yaml`**: Nav2 stack parameters
  - AMCL (Adaptive Monte Carlo Localization) settings
  - Local/Global costmap configurations
  - DWB (Dynamic Window Approach) planner parameters
  - Controller server settings

#### Maps
- **`assessment_map.yaml`** / **`assessment_map.pgm`**: Pre-generated SLAM map of the environment

#### Python Nodes

**`mission_control.py`** ⭐ **(PRIMARY NODE)**
- **Role**: Central command node orchestrating the entire mission
- **Functionality**:
  1. **Vision-Based Approach**: 
     - Subscribes to `/vision/tracking_data` from vision detector
     - Calculates steering commands to center blue sphere in camera view
     - Uses proportional control for smooth tracking
  2. **Obstacle Avoidance**:
     - Processes Lidar `/scan` data
     - Applies potential field obstacle repulsion forces
     - Prevents collisions during sphere approach
  3. **Autonomous Navigation**:
     - Interfaces with Nav2 via `BasicNavigator`
     - Plans and executes paths to "pen" goal position
     - Monitors navigation status and recovery behaviors
  4. **Interactive Control**:
     - Accepts user commands to switch between modes
     - Modes: `find` (approach sphere), `go` (navigate to pen)
     - Publishes velocity commands to `/diff_drive_base_controller/cmd_vel`
  5. **State Machine**:
     - Tracks mission states (SEARCHING, APPROACHING, NAVIGATING, COMPLETE)
     - Handles recovery behaviors when stuck or target lost

**`vision_detector.py`**
- **Role**: Blue sphere detection using OpenCV
- **Functionality**:
  1. Subscribes to `/camera/image_raw`
  2. Converts to HSV color space
  3. Applies blue color thresholding (HSV range: 110-130)
  4. Detects largest blue contour (filters noise <100px)
  5. Calculates:
     - **Steering Error** (x): Normalized horizontal offset (-1.0 to 1.0)
     - **Proximity** (y): Sphere width ratio (0.0 far, 1.0 close)
     - **Detection Flag** (z): 1.0 if found, 0.0 if lost
  6. Publishes tracking data to `/vision/tracking_data`

**`go_to_pen.py`**
- **Role**: Simplified navigation node for pen return
- **Functionality**: Sends single Nav2 goal to pen location

#### Launch Files

**`navigation.launch.py`**
```bash
ros2 launch spherebot_navigation navigation.launch.py
```
- Launches complete Nav2 navigation stack
- Loads pre-generated map (`assessment_map.yaml`)
- Starts AMCL localization, planners, and controllers
- Opens RViz with Nav2 visualization
- **Primary navigation launcher** - used in main mission workflow

**`go_to_pen.launch.py`**
```bash
ros2 launch spherebot_navigation go_to_pen.launch.py
```
- Quick launcher for automated pen navigation
- Automatically sends Nav2 goal to pen location
- Useful for testing return-to-home functionality

---

### `spherebot_slam`

**Purpose**: SLAM (Simultaneous Localization and Mapping) for environment mapping.

**Key Components**:
- **SLAM Configuration**: Parameters for SLAM Toolbox
- **Map Generation**: Tools to create and save `.pgm` map files
- **Localization**: Map-based localization for autonomous navigation

**Usage**: Used during initial setup to generate the `assessment_map` used by Nav2

#### Launch Files

**`slam.launch.py`**
```bash
ros2 launch spherebot_slam slam.launch.py
```
- Launches SLAM Toolbox with RViz visualization
- Robot builds map in real-time as it explores environment
- Uses Lidar data to create occupancy grid map
- Requires robot to be running in Gazebo (use `spherebot_sim.launch.py` first)

**Save Generated Map**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2/ros2_ws/src/spherebot_slam/maps/assessment_map
```
- Saves the current SLAM-generated map to disk
- Creates two files: `assessment_map.yaml` and `assessment_map.pgm`
- Run this command after exploring the entire environment
- Saved map can then be used by Nav2 for autonomous navigation

---

### `spherebot_teleop`

**Purpose**: Manual keyboard control for robot teleoperation.

**Key Components**:

**`keyboard_teleop.py`**
- **Role**: Human-in-the-loop manual control
- **Control Scheme**:
  ```
     w         - Move forward
  a  s  d      - Turn left / Stop / Turn right
     x         - Move backward
  
     c         - Close gates (capture sphere)
     o         - Open gates (release sphere)
  ```
- **Publishers**:
  - `/diff_drive_base_controller/cmd_vel` (TwistStamped): Velocity commands
  - `/left_gate_controller/commands` (Float64MultiArray): Left gate position
  - `/right_gate_controller/commands` (Float64MultiArray): Right gate position
- **Gate Positions**:
  - Open: 0.0 rad
  - Closed: 1.57 rad (90°)

**When to Use Teleop**:
1. **Docking Blue Sphere**: Fine positioning to catch sphere
2. **Collecting Other Spheres**: Green and red spheres (no vision system)
3. **Recovery Maneuvers**: When robot gets stuck in Nav2 costmap boundaries
4. **Gate Control**: Opening/closing capture mechanism

#### Run Command

```bash
ros2 run spherebot_teleop keyboard
```
- Starts keyboard teleoperation node
- Takes over terminal input for control
- Press CTRL-C to exit and return terminal control
- Can run alongside autonomous nodes for manual intervention

---

## How to Run the Project

This section provides step-by-step instructions to launch and operate SphereBOT. **You will need 5 terminals**.

### Prerequisites

Before running, ensure:
1. ROS2 Jazzy (or compatible version) is installed
2. Workspace is built: `cd ~/ros2/ros2_ws && colcon build`
3. Workspace is sourced: `source ~/ros2/ros2_ws/install/setup.bash`
4. Gazebo Harmonic is installed and configured

> **TIP**: Add `source ~/ros2/ros2_ws/install/setup.bash` to your `~/.bashrc` to automatically source on terminal startup.

---

### Step-by-Step Launch Procedure

#### **Terminal 1: Launch Simulation Environment**

```bash
source ~/ros2/ros2_ws/install/setup.bash
ros2 launch spherebot_gazebo spherebot_sim.launch.py
```

**What This Does**:
- Starts Gazebo simulation with the assessment world
- Spawns SphereBOT at the starting position
- Spawns three random spheres (blue, green, red) in random locations
- Initializes robot sensors (Lidar, Camera, IMU)
- Starts robot controllers (differential drive, gate controllers)
- Launches ROS-Gazebo bridge for sensor/actuator communication
- Starts EKF localization node for sensor fusion
- Opens RViz for robot visualization

**What You Should See**:
- Gazebo window showing a room environment with your robot and three spheres
- RViz window showing robot model, Lidar scans, and camera view

---

#### **Terminal 2: Launch Navigation Stack**

```bash
source ~/ros2/ros2_ws/install/setup.bash
ros2 launch spherebot_navigation navigation.launch.py
```

**What This Does**:
- Loads the pre-generated map (`assessment_map.yaml`)
- Starts Nav2 navigation stack:
  - **Map Server**: Provides static map
  - **AMCL**: Localizes robot on the map using particle filter
  - **Planner Server**: Generates global paths to goals
  - **Controller Server**: Executes local trajectory control
  - **Behavior Tree Navigator**: Coordinates navigation behaviors
  - **Recoveries**: Handles stuck situations
- Opens RViz with Nav2 visualization:
  - Global costmap (blue)
  - Local costmap (red)
  - Planned paths
  - Robot pose estimate

**What You Should See**:
- New RViz window with Nav2 panels
- Green particle cloud showing AMCL localization estimates
- Costmaps overlaying the map

**Important**: Wait ~10-15 seconds for AMCL to converge before proceeding.

---

#### **Terminal 3: Start Vision Detector**

```bash
source ~/ros2/ros2_ws/install/setup.bash
ros2 run spherebot_navigation vision_detector
```

**What This Does**:
- Subscribes to robot's front camera (`/camera/image_raw`)
- Processes images in real-time to detect blue sphere
- Uses HSV color filtering to isolate blue objects
- Calculates:
  - Steering correction (how far left/right the sphere is)
  - Distance estimate (based on sphere size in image)
- Publishes tracking data to `/vision/tracking_data`

**What You Should See**:
- Terminal output (minimal unless errors occur)
- Vision system is now actively tracking blue sphere

**Verification**: You can check if vision is working:
```bash
ros2 topic echo /vision/tracking_data
```
You should see Point messages with x, y, z values when blue sphere is visible.

---

#### **Terminal 4: Start Mission Control** ⭐

```bash
source ~/ros2/ros2_ws/install/setup.bash
ros2 run spherebot_navigation mission_control
```

**What This Does**:
- Launches the central command node
- Integrates vision tracking with obstacle avoidance
- Provides interactive control interface
- Manages autonomous navigation to pen

**What You Should See**:
```
Mission Control Active
Commands:
  'find' - Approach blue sphere (vision + obstacle avoid)
  'go'   - Navigate to pen (Nav2 autonomous)
  'q'    - Quit
>
```

**Interactive Commands**:

1. **`find`** - Vision-Based Sphere Approach
   - Robot will turn to center blue sphere in camera view
   - Drives forward while avoiding obstacles
   - Uses Lidar for collision prevention
   - Robot will approach until sphere is close (proximity threshold)
   - **Important**: This does NOT close gates - use teleop for final docking

2. **`go`** - Autonomous Navigation to Pen
   - Sends goal to Nav2 stack (pen location)
   - Robot autonomously plans path and navigates
   - Monitors progress and handles recovery
   - **When to use**: After manually collecting spheres with gates closed

**How Mission Control Works**:

**Vision-Based Approach (`find` mode)**:
- **Input Data**:
  - Vision tracking data (steering error, proximity)
  - Lidar scan (obstacle distances)
  - Gate state (open/closed)
- **Control Logic**:
  - Proportional steering: Turns to center sphere in view
  - Forward speed: Adjusts based on proximity (slower when close)
  - Obstacle repulsion: Applies forces to avoid collisions
  - Dead zone: Stops adjusting when sphere is centered
- **State Management**:
  - SEARCHING: Rotating to find blue sphere
  - APPROACHING: Driving toward sphere with obstacle avoidance
  - CLOSE: Near sphere, ready for manual capture

**Nav2 Navigation (`go` mode)**:
- Uses BasicNavigator API to send goal pose
- Monitors TaskResult for success/failure
- Handles recovery behaviors if stuck
- Returns control when goal reached or failed

---

#### **Terminal 5: Manual Teleoperation**

```bash
source ~/ros2/ros2_ws/install/setup.bash
ros2 run spherebot_teleop keyboard
```

**What This Does**:
- Provides keyboard control for manual driving
- Allows gate control for sphere capture
- Used when autonomy cannot handle precision tasks

**What You Should See**:
```
Control Your Spherebot!
---------------------------
Moving around:
   w
a  s  d
   x

Gates:
   c : Close Gates (Catch)
   o : Open Gates (Release)

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
s   : force stop

CTRL-C to quit
```

**When to Use Teleop**:

1. **Docking Blue Sphere**:
   - After `find` command gets robot close to blue sphere
   - Use `w`/`a`/`d` keys to position robot perfectly
   - Press `c` to close gates and capture sphere
   - Verify sphere is inside gates visually in Gazebo

2. **Collecting Green and Red Spheres**:
   - No vision system for these colors
   - Manually drive to sphere location (check Gazebo)
   - Position robot to push sphere between gates
   - Press `c` to close gates and capture

3. **Escaping Costmap Boundaries**:
   - If Nav2 fails due to costmap issues
   - Use `w`/`x`/`a`/`d` to manually navigate out
   - Watch local costmap in RViz to see clear areas

4. **Final Delivery**:
   - Press 'go' in misson control terminal to send robot to pen goal
   - Press `o` to open gates and release spheres once it reaches home

**Important**: Keep this terminal active throughout the mission for quick manual intervention.

---

### Complete Mission Workflow

**Objective**: Collect all three spheres and deliver to pen.

**Recommended Procedure**:

1. **Launch All Systems** (Terminals 1-5 as described above)

2. **Collect Blue Sphere** (Autonomous + Manual):
   ```
   Terminal 4 (mission_control):
   ```
   - Robot autonomously approaches blue sphere
   - Watch in Gazebo as robot drives toward sphere
   - When close, switch to Terminal 5 (teleop)
   - Fine-tune position with `w`/`a`/`d` keys
   - Press `c` to close gates and capture sphere
   - SphereBot will return to home position

3. **Navigate to Green Sphere** (Manual):
   - Terminal 5 (teleop): Drive to green sphere location
   - Position robot to align sphere between gates
   - Adjust with `w`/`a`/`d` keys
   - Verify sphere is between gates in Gazebo
   - If gates were opened, press `c` to close
   - Type 'go' in Terminal 4 (mission_control)
   - SphereBot will return to home position

4. **Navigate to Red Sphere** (Manual):
   - Terminal 5 (teleop): Drive to red sphere location
   - Repeat positioning process
   - Press `c` to close gates if needed
   - Type 'go' in Terminal 4 (mission_control)
   - SphereBot will return to home position

5. **Return to Pen** (Autonomous or Manual):
   - **Option A** (Autonomous):
     ```
     Terminal 4 (mission_control): Type 'go' and press Enter
     ```
     - Robot autonomously navigates to pen
     - Monitor progress in Nav2 RViz
   - **Option B** (Manual):
     - Terminal 5 (teleop): Drive to pen area
     - Use Gazebo overhead view to locate pen

6. **Deliver Spheres**:
   - Position robot inside or at pen entrance
   - Terminal 5 (teleop): Press `o` to open gates
   - Spheres roll out into pen area
   - **Mission Complete** 

---

### Troubleshooting


**Problem**: Nav2 navigation fails or robot gets stuck
- **Check**: AMCL localization has converged (green particle cloud in RViz)
- **Check**: Goal position is in free space (not in obstacle on costmap)
- **Solution**: Use teleop to move robot to clear area, retry navigation

**Problem**: Gates don't close/open with `c`/`o` keys
- **Check**: Gate controllers are running: `ros2 topic list | grep gate`
- **Verify**: `/left_gate_controller/commands` and `/right_gate_controller/commands` exist
- **Solution**: Restart simulation (Terminal 1)

**Problem**: Robot spins in place during `find` mode
- **Cause**: Blue sphere not detected by vision
- **Solution**: Check Gazebo for blue sphere location, use teleop to face it

**Problem**: Simulation crashes or freezes
- **Solution**: Close all terminals, restart from Terminal 1

**Problem**: "Costmap out of bounds" errors
- **Cause**: Robot position outside pre-mapped area
- **Solution**: Use teleop to move robot back to mapped region (watch RViz map)

---

## Project Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         GAZEBO SIMULATION                        │
│  - Assessment World                                              │
│  - SphereBOT Robot                                               │
│  - 3 Spheres (Random Spawn)                                      │
└────────────────┬────────────────────────────┬────────────────────┘
                 │                            │
         ┌───────▼────────┐         ┌─────────▼──────────┐
         │  ROS-Gz Bridge │         │  ros2_control      │
         │  - Camera      │         │  - Diff Drive      │
         │  - Lidar       │         │  - Gate Controllers│
         │  - IMU         │         └─────────┬──────────┘
         └───────┬────────┘                   │
                 │                            │
    ┌────────────▼────────────────────────────▼─────────────┐
    │              ROBOT LOCALIZATION (EKF)                  │
    │       Fuses: Odometry + IMU → /odometry/filtered       │
    └────────────┬───────────────────────────────────────────┘
                 │
    ┌────────────▼────────────────────────────────────────┐
    │                    NAV2 STACK                        │
    │  - Map Server (Pre-generated Map)                    │
    │  - AMCL Localization                                 │
    │  - Planner Server (Global Path)                      │
    │  - Controller Server (Local Trajectory)              │
    └────────────┬───────────────────────────────────────┬─┘
                 │                                       │
    ┌────────────▼──────────┐           ┌───────────────▼────────┐
    │   VISION DETECTOR      │           │   MISSION CONTROL      │
    │   - Blue Sphere Track  │ ─────────▶│   - Vision Integration │
    │   - OpenCV Processing  │           │   - Obstacle Avoidance │
    │                        │           │   - Nav2 Interface     │
    └────────────────────────┘           │   - Interactive Mode   │
                                         └───────────┬────────────┘
                                                     │
                                         ┌───────────▼────────────┐
                                         │   VELOCITY COMMANDS    │
                                         │   /cmd_vel             │
                                         └────────────────────────┘

                        ┌────────────────────────────┐
                        │   KEYBOARD TELEOP          │
                        │   - Manual Control         │
                        │   - Gate Control           │
                        └────────────────────────────┘
```

---

## Key Topics Reference

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Front camera RGB stream |
| `/scan` | sensor_msgs/LaserScan | 2D Lidar distance measurements |
| `/imu/data` | sensor_msgs/Imu | IMU acceleration/gyro data |
| `/vision/tracking_data` | geometry_msgs/Point | Blue sphere tracking (x=steering, y=proximity, z=detected) |
| `/diff_drive_base_controller/cmd_vel` | geometry_msgs/TwistStamped | Velocity commands to robot |
| `/diff_drive_base_controller/odom` | nav_msgs/Odometry | Wheel odometry |
| `/left_gate_controller/commands` | std_msgs/Float64MultiArray | Left gate position command |
| `/right_gate_controller/commands` | std_msgs/Float64MultiArray | Right gate position command |
| `/goal_pose` | geometry_msgs/PoseStamped | Nav2 goal position |

---

## Dependencies

**ROS2 Packages**:
- `ros2_control` / `ros2_controllers`
- `gz_ros2_control` (Gazebo integration)
- `nav2_bringup` (Navigation stack)
- `slam_toolbox` (SLAM capabilities)
- `robot_localization` (EKF fusion)
- `ros_gz_bridge` / `ros_gz_sim` (Gazebo-ROS bridge)

**Python Libraries**:
- `opencv-python` / `cv-bridge` (Vision processing)
- `numpy` (Numerical operations)

---

## Future Enhancements

- **Multi-Sphere Vision**: Extend vision detector to detect green and red spheres
- **Autonomous Collection**: Automated gate control sequence
- **Dynamic Obstacles**: Handle moving obstacles in environment
- **Sim-to-Real**: Transfer to physical robot hardware

---

## Credits

**Author**: Faseeh Mohammed 
**MSIS**: M01088120 
**Email**: faseehmohammed@outlook.com  
**Framework**: ROS2 Jazzy  
**Simulator**: Gazebo Harmonic  

---

## References

- https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Building-a-Movable-Robot-Model-with-URDF.html
- https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html
- https://control.ros.org/jazzy/doc/ros2_control_demos/example_2/doc/userdoc.html
- https://docs.ros.org/en/ros2_packages/jazzy/api/nav2_simple_commander/
- https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/
- https://automaticaddison.com/autonomous-navigation-for-a-mobile-robot-using-ros-2-jazzy/
- https://www.youtube.com/watch?v=wZV1PoZ4fyk
- https://www.youtube.com/watch?v=V9ztoMuSX8w

---

**Happy Robot Navigation**
