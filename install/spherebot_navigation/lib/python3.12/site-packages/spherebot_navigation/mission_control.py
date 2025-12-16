import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import JointState, LaserScan

# --- CONFIGURATION ---
INITIAL_X = -3.69
INITIAL_Y = 0.0
INITIAL_YAW = 0.0

TARGET_VISUAL_WIDTH = 0.40
OBSTACLE_DIST_THRESH = 0.8  
AVOID_WEIGHT = 2.0          
SEARCH_SPEED = 0.4          
# EMERGENCY_DIST: Distance to trigger the "Back-Out" maneuver
EMERGENCY_DIST = 0.45       
VISION_DEADZONE = 0.10      
MIN_FORWARD_SPEED = 0.08    
MAX_ANGULAR_VEL = 1.0       

# TIMING SETTINGS
RECOVERY_DURATION = 1.5     # Time to clear obstacle after losing ball
BACKUP_DURATION = 2.0       # [NEW] Time to reverse when stuck

class MissionListener(Node):
    def __init__(self):
        super().__init__('mission_listener')
        self.track_error = 0.0
        self.visual_width = 0.0
        self.target_detected = False
        self.gates_closed = False
        
        # Obstacle Forces
        self.avoid_turn = 0.0
        self.emergency_stop = False
        
        # State Memory
        self.last_avoidance_direction = 0.0
        self.is_avoiding = False
        self.recovery_start_time = 0.0 
        self.backup_start_time = 0.0  # [NEW] Timer for backing up
        
        self.create_subscription(Point, '/vision/tracking_data', self.vision_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)

    def vision_cb(self, msg):
        if msg.z == 1.0:
            self.target_detected = True
            self.track_error = msg.x
            self.visual_width = msg.y
        else:
            self.target_detected = False

    def scan_cb(self, msg):
        ranges = msg.ranges
        if len(ranges) < 360: return
        
        # Front-Left: 180 to 230 (+50 deg)
        front_left_sector = ranges[180:230]
        # Front-Right: 130 to 180 (-50 deg)
        front_right_sector = ranges[130:180]
        
        # Emergency Cone: +/- 15 degrees (Slightly wider to catch corners)
        front_cone = ranges[165:195]
        
        emergency_zone = [r for r in front_cone if 0.05 < r < EMERGENCY_DIST]
        left_valid = [r for r in front_left_sector if 0.05 < r < OBSTACLE_DIST_THRESH]
        right_valid = [r for r in front_right_sector if 0.05 < r < OBSTACLE_DIST_THRESH]
        
        # 1. EMERGENCY CHECK
        if len(emergency_zone) > 0:
            self.emergency_stop = True
            # Force a slight turn while backing up to help un-wedge
            self.avoid_turn = 0.0 
        else:
            self.emergency_stop = False
        
        turn_cmd = 0.0
        
        # 2. STANDARD AVOIDANCE
        if len(left_valid) > 0:
            min_dist = min(left_valid)
            push = (OBSTACLE_DIST_THRESH - min_dist) / OBSTACLE_DIST_THRESH
            turn_cmd -= push * AVOID_WEIGHT

        if len(right_valid) > 0:
            min_dist = min(right_valid)
            push = (OBSTACLE_DIST_THRESH - min_dist) / OBSTACLE_DIST_THRESH
            turn_cmd += push * AVOID_WEIGHT
            
        self.avoid_turn = turn_cmd
        
        if abs(turn_cmd) > 0.2:
            self.is_avoiding = True
            self.last_avoidance_direction = turn_cmd 
        else:
            self.is_avoiding = False

    def joint_cb(self, msg):
        try:
            if 'left_gate_joint' in msg.name:
                idx = msg.name.index('left_gate_joint')
                if abs(msg.position[idx]) > 0.5:
                    self.gates_closed = True
                else:
                    self.gates_closed = False
        except:
            pass

def create_pose(navigator, x, y, yaw_rad):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw_rad / 2)
    pose.pose.orientation.w = math.cos(yaw_rad / 2)
    return pose

def main():
    rclpy.init()
    listener = MissionListener()
    navigator = BasicNavigator()
    
    vel_pub = listener.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)

    # Init Pose
    initial_pose = create_pose(navigator, INITIAL_X, INITIAL_Y, INITIAL_YAW)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    wp_sphere = create_pose(navigator, 1.5, 0.0, 0.0)
    
    # PHASE 1: SEARCH
    print(">>> PHASE 1: Moving to Search Area...")
    navigator.goToPose(wp_sphere)

    while not navigator.isTaskComplete():
        rclpy.spin_once(listener, timeout_sec=0.1)
        if listener.target_detected:
            print(f"\n!!! TARGET SPOTTED !!!")
            navigator.cancelTask()
            break

    # PHASE 1.5: SMART CHASE
    print(f">>> CHASING TARGET (Smart Recovery Mode)...")
    
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = 'base_link'
    
    while rclpy.ok():
        rclpy.spin_once(listener, timeout_sec=0.1)
        twist_msg.header.stamp = listener.get_clock().now().to_msg()
        current_time = listener.get_clock().now().nanoseconds / 1e9
        
        # SUCCESS CONDITION
        if listener.target_detected and listener.visual_width >= TARGET_VISUAL_WIDTH:
            print(">>> TARGET REACHED. STOPPING.")
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            vel_pub.publish(twist_msg)
            break
        
        # ==========================================================
        # PRIORITY 1: EMERGENCY BACK-OUT MANEUVER
        # ==========================================================
        
        # If currently too close, reset the backup timer
        if listener.emergency_stop:
            listener.backup_start_time = current_time
            print("!!! OBSTACLE INSIDE GRIPPER! INITIATING BACKOUT...")

        # If the timer is active (meaning we were too close recently), keep backing up
        # This ensures we back up for a full 2 seconds even if the sensor clears instantly
        if (current_time - listener.backup_start_time) < BACKUP_DURATION:
            twist_msg.twist.linear.x = -0.2  # REVERSE SPEED
            
            # Slight turn helps unhook the U-arm if caught
            # We turn opposite to the last known obstacle direction
            twist_msg.twist.angular.z = 0.5 if listener.last_avoidance_direction < 0 else -0.5
            
            vel_pub.publish(twist_msg)
            continue
            
        # ==========================================================
        
        # PRIORITY 2: BALL LOST -> RECOVERY LOGIC
        if not listener.target_detected:
            if listener.is_avoiding or (current_time - listener.recovery_start_time < RECOVERY_DURATION):
                if listener.recovery_start_time == 0:
                    listener.recovery_start_time = current_time
                    print("!!! LOST TARGET NEAR OBSTACLE. CLEARING ZONE...")

                # Drive FORWARD and curve away
                twist_msg.twist.linear.x = 0.15
                recovery_turn = 0.5 if listener.last_avoidance_direction > 0 else -0.5
                twist_msg.twist.angular.z = recovery_turn
                
            else:
                listener.recovery_start_time = 0
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = SEARCH_SPEED
            
            vel_pub.publish(twist_msg)
            continue
            
        else:
            listener.recovery_start_time = 0

        # PRIORITY 3: CHASE & AVOID
        if abs(listener.track_error) < VISION_DEADZONE:
            vision_turn = 0.0
        else:
            vision_turn = listener.track_error * 1.5

        avoidance_turn = listener.avoid_turn
        
        final_turn = vision_turn + avoidance_turn
        final_turn = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, final_turn))
        
        if abs(final_turn) > 0.8:
            linear_speed = 0.05
        else:
            linear_speed = 0.15

        twist_msg.twist.linear.x = max(MIN_FORWARD_SPEED, linear_speed)
        twist_msg.twist.angular.z = final_turn
        
        vel_pub.publish(twist_msg)

    # PHASE 2: MANUAL DOCKING
    print("\n>>> PHASE 2: MANUAL DOCKING. CLOSE GATES ('c').")
    while not listener.gates_closed:
        rclpy.spin_once(listener, timeout_sec=0.1)
        time.sleep(0.1)

    print(">>> GATES CLOSED! RETURNING HOME...")
    time.sleep(1.0)

    # PHASE 3: RETURN HOME (First time)
    wp_pen = create_pose(navigator, 0.1, 3.5, 1.57)
    
    print(f">>> NAVIGATING TO HOME (Goal: {wp_pen.pose.position.x}, {wp_pen.pose.position.y})...")
    navigator.goToPose(wp_pen)

    while not navigator.isTaskComplete():
        rclpy.spin_once(listener, timeout_sec=0.1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(">>> HOME REACHED!")
    else:
        print(">>> NAVIGATION FAILED!")

    # PHASE 4: INTERACTIVE COMMAND LOOP
    print("\n" + "="*60)
    print(">>> INTERACTIVE MODE")
    print(">>> Type 'go' to navigate home from current position")
    print(">>> Type 'exit' to quit the program")
    print("="*60)
    
    import threading
    import sys
    
    user_command = [None]  # Use list to share between threads
    command_lock = threading.Lock()
    
    def input_thread():
        """Thread to handle user input without blocking ROS"""
        while rclpy.ok():
            try:
                cmd = input(">>> Enter command: ").strip().lower()
                with command_lock:
                    user_command[0] = cmd
            except EOFError:
                break
    
    # Start input thread
    input_handler = threading.Thread(target=input_thread, daemon=True)
    input_handler.start()
    
    # Main command processing loop
    while rclpy.ok():
        # Keep ROS alive
        rclpy.spin_once(listener, timeout_sec=0.1)
        
        # Check for user command
        with command_lock:
            cmd = user_command[0]
            user_command[0] = None  # Clear after reading
        
        if cmd == 'go':
            print("\n>>> NAVIGATING TO HOME FROM CURRENT POSITION...")
            navigator.goToPose(wp_pen)
            
            while not navigator.isTaskComplete():
                rclpy.spin_once(listener, timeout_sec=0.1)
            
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(">>> HOME REACHED!")
            else:
                print(">>> NAVIGATION FAILED!")
            
            print("\n>>> Waiting for next command...")
            
        elif cmd == 'exit':
            print("\n>>> EXITING PROGRAM...")
            break
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()