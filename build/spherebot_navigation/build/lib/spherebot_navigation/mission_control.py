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

TARGET_VISUAL_WIDTH = 0.20
OBSTACLE_DIST_THRESH = 1.2  # React if obstacle is closer than 1.2m (robot is 0.66m wide + safety)
AVOID_WEIGHT = 3.5          # How hard to turn away from obstacles (reduced to prevent over-steering)
SEARCH_SPEED = 0.4          # Rotation speed when searching for lost ball
EMERGENCY_DIST = 0.5        # Emergency stop distance
VISION_DEADZONE = 0.15      # Ignore small tracking errors to prevent wiggling
MIN_FORWARD_SPEED = 0.08    # Minimum speed when ball detected (prevents getting stuck)
MAX_ANGULAR_VEL = 1.2       # Cap angular velocity to prevent excessive turns

class MissionListener(Node):
    def __init__(self):
        super().__init__('mission_listener')
        self.track_error = 0.0
        self.visual_width = 0.0
        self.target_detected = False
        self.gates_closed = False
        
        # Obstacle Forces (Left vs Right)
        self.avoid_turn = 0.0
        self.emergency_stop = False  # Flag for very close obstacles
        
        # Ball tracking memory for search mode
        self.last_track_error = 0.0
        
        # Directional memory for smart search
        self.last_avoidance_direction = 0.0  # Positive = turned left, Negative = turned right
        self.was_avoiding = False  # Track if we were avoiding in the last cycle
        
        # Subscriptions
        self.create_subscription(Point, '/vision/tracking_data', self.vision_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        # Lidar is CRITICAL for obstacle avoidance
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)

    def vision_cb(self, msg):
        if msg.z == 1.0:
            self.target_detected = True
            self.track_error = msg.x
            self.visual_width = msg.y
            # Remember last ball direction for search mode
            self.last_track_error = msg.x
        else:
            self.target_detected = False

    def scan_cb(self, msg):
        # CORRECTED LIDAR INDEXING:
        # Lidar: min_angle=-3.14, max_angle=3.14, 360 samples
        # Index 0   = -PI (-180°) = back-right
        # Index 90  = -PI/2 (-90°) = right side
        # Index 180 = 0 (0°) = FRONT
        # Index 270 = +PI/2 (+90°) = left side
        
        ranges = msg.ranges
        num_readings = len(ranges)  # Should be 360
        
        if num_readings == 0:
            return
        
        # Front is at index 180 (0 radians)
        # Scan wider arc: 160° forward (80° left + 80° right)
        # Left sector: indices 180 to 260 (0° to +80°)
        # Right sector: indices 100 to 180 (-80° to 0°)
        
        front_left_sector = ranges[180:260] if len(ranges) > 260 else ranges[180:]
        front_right_sector = ranges[100:180]
        
        # Also check narrow front cone for emergency stop
        front_cone = ranges[170:190] if len(ranges) > 190 else ranges[170:]
        
        # Filter valid readings in different zones
        emergency_zone = [r for r in front_cone if 0.05 < r < EMERGENCY_DIST]
        left_valid = [r for r in front_left_sector if 0.05 < r < OBSTACLE_DIST_THRESH]
        right_valid = [r for r in front_right_sector if 0.05 < r < OBSTACLE_DIST_THRESH]
        
        # EMERGENCY STOP: Very close obstacle directly ahead
        if len(emergency_zone) > 0:
            self.emergency_stop = True
            self.avoid_turn = 0.0
            return
        else:
            self.emergency_stop = False
        
        turn_cmd = 0.0
        
        # 1. If Obstacle on LEFT -> Turn RIGHT (Negative Z)
        if len(left_valid) > 0:
            min_dist = min(left_valid)
            # Stronger push if closer
            push = (OBSTACLE_DIST_THRESH - min_dist) / OBSTACLE_DIST_THRESH
            turn_cmd -= push * AVOID_WEIGHT

        # 2. If Obstacle on RIGHT -> Turn LEFT (Positive Z)
        if len(right_valid) > 0:
            min_dist = min(right_valid)
            push = (OBSTACLE_DIST_THRESH - min_dist) / OBSTACLE_DIST_THRESH
            turn_cmd += push * AVOID_WEIGHT
            
        self.avoid_turn = turn_cmd

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

    # Init Pose & Waypoints
    initial_pose = create_pose(navigator, INITIAL_X, INITIAL_Y, INITIAL_YAW)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    wp_sphere = create_pose(navigator, 1.5, 0.0, 0.0)
    wp_pen = create_pose(navigator, 7.0, 34.0, 58.0)

    # --- PHASE 1: SEARCH ---
    print(">>> PHASE 1: Moving to Search Area...")
    navigator.goToPose(wp_sphere)

    while not navigator.isTaskComplete():
        rclpy.spin_once(listener, timeout_sec=0.1)
        if listener.target_detected:
            print(f"\n!!! TARGET SPOTTED !!!")
            navigator.cancelTask()
            break

    # --- PHASE 1.5: SMART CHASE WITH OBSTACLE AVOIDANCE ---
    print(f">>> CHASING TARGET WITH OBSTACLE AVOIDANCE...")
    
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = 'base_link'
    
    while rclpy.ok():
        rclpy.spin_once(listener, timeout_sec=0.1)
        twist_msg.header.stamp = listener.get_clock().now().to_msg()
        
        # Check if target reached
        if listener.target_detected and listener.visual_width >= TARGET_VISUAL_WIDTH:
            print(">>> TARGET REACHED. STOPPING.")
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            vel_pub.publish(twist_msg)
            break
        
        # --- 4-PRIORITY BLENDED CONTROL ---
        
        # PRIORITY 1: EMERGENCY STOP
        if listener.emergency_stop:
            print("!!! EMERGENCY STOP - Obstacle too close!")
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            vel_pub.publish(twist_msg)
            continue
        
        # PRIORITY 2: SEARCH MODE (ball lost)
        if not listener.target_detected:
            print(f"Ball lost! Searching... (last avoidance: {listener.last_avoidance_direction:.2f})")
            twist_msg.twist.linear.x = 0.0  # Don't move forward blind
            
            # SMART SEARCH: Rotate OPPOSITE to the last avoidance direction
            # If we steered left (+) to avoid, search right (-), and vice versa
            if abs(listener.last_avoidance_direction) > 0.3:
                # We had a significant avoidance maneuver - search opposite way
                search_direction = -1.0 * SEARCH_SPEED * (1.0 if listener.last_avoidance_direction > 0 else -1.0)
            else:
                # No clear avoidance direction - default clockwise search
                search_direction = SEARCH_SPEED
            
            # Apply search + any current obstacle avoidance
            twist_msg.twist.angular.z = search_direction + listener.avoid_turn
            vel_pub.publish(twist_msg)
            continue
        
        # PRIORITY 3 & 4: CHASE WITH OBSTACLE AVOIDANCE (blended)
        
        # Apply dead zone to vision tracking (prevents micro-oscillations)
        if abs(listener.track_error) < VISION_DEADZONE:
            vision_turn = 0.0
        else:
            # Attraction: Turn towards ball (reduced gain for smoother control)
            vision_turn = listener.track_error * 1.2
        
        # Repulsion: Turn away from obstacles
        avoidance_turn = listener.avoid_turn
        
        # Track if we're currently avoiding obstacles
        if abs(avoidance_turn) > 0.5:
            listener.was_avoiding = True
            listener.last_avoidance_direction = avoidance_turn  # Remember which way we turned
        else:
            listener.was_avoiding = False
        
        # Blend both influences
        final_turn = vision_turn + avoidance_turn
        
        # Cap angular velocity to prevent 180° spins
        final_turn = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, final_turn))
        
        # Adaptive speed based on obstacle proximity
        if abs(avoidance_turn) > 2.0:
            # Strong avoidance needed - creep forward
            linear_speed = 0.05
        elif abs(avoidance_turn) > 1.0:
            # Moderate avoidance - slow down
            linear_speed = 0.10
        else:
            # Clear path - normal chase speed
            linear_speed = 0.15
        
        # Ensure minimum forward speed when ball is detected (prevents getting stuck)
        twist_msg.twist.linear.x = max(MIN_FORWARD_SPEED, linear_speed)
        twist_msg.twist.angular.z = final_turn
        
        vel_pub.publish(twist_msg)

    # --- PHASE 2: MANUAL DOCKING ---
    print("\n>>> PHASE 2: MANUAL DOCKING. CLOSE GATES ('c').")
    while not listener.gates_closed:
        rclpy.spin_once(listener, timeout_sec=0.1)
        time.sleep(0.1)

    print(">>> RETURNING HOME.")
    time.sleep(1.0)

# ==========================
    # PHASE 3: RETURN
    # ==========================
    # Use coordinates INSIDE the 8x8m box
    # Example: Top-Left corner of the arena
    wp_pen = create_pose(navigator, 0.1, 2.5, 1.57) 
    
    print(f">>> RETURNING HOME (Goal: {wp_pen.pose.position.x}, {wp_pen.pose.position.y})...")
    navigator.goToPose(wp_pen)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # Optional: Print distance remaining
        # if feedback:
        #     print(f"Distance remaining: {feedback.distance_remaining:.2f}")
        rclpy.spin_once(listener, timeout_sec=0.1)

    # CHECK THE ACTUAL RESULT
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print(">>> MISSION COMPLETE: GOAL REACHED!")
    elif result == TaskResult.CANCELED:
        print(">>> MISSION FAILED: GOAL WAS CANCELED!")
    elif result == TaskResult.FAILED:
        print(">>> MISSION FAILED: GOAL UNREACHABLE OR ABORTED!")
        print("    (Check if goal is inside the map walls)")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()