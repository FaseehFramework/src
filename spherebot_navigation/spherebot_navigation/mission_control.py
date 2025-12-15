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
OBSTACLE_DIST_THRESH = 0.7  # React if obstacle is closer than 0.7m
AVOID_WEIGHT = 2.5          # How hard to turn away from obstacles

class MissionListener(Node):
    def __init__(self):
        super().__init__('mission_listener')
        self.track_error = 0.0
        self.visual_width = 0.0
        self.target_detected = False
        self.gates_closed = False
        
        # Obstacle Forces (Left vs Right)
        self.avoid_turn = 0.0 
        
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
        else:
            self.target_detected = False

    def scan_cb(self, msg):
        # Lidar ranges are usually [0] = Front, [90] = Left, [270] = Right (depending on device)
        # Gazebo usually maps 0 to -Pi (Right/Back) and Last to +Pi (Left/Back)
        # We need to find the "Forward Left" and "Forward Right" sectors.
        
        ranges = msg.ranges
        num_readings = len(ranges)
        
        # Define sectors (assuming 360 readings, 0 is back/right, center is front)
        # Adjust indices based on your specific Lidar plugin. 
        # Standard Gazebo 2D Lidar often has 0 at Front or Right.
        # Let's assume standard ROS convention: 0 is Front? Or Min Angle?
        # Your URDF says min -3.14, max 3.14. So Index 0 is Back (-PI), Middle is Front (0).
        
        mid_idx = num_readings // 2
        
        # Slice Forward-Left (0 to 45 deg left)
        left_sector = ranges[mid_idx : mid_idx + 40]
        # Slice Forward-Right (0 to 45 deg right)
        right_sector = ranges[mid_idx - 40 : mid_idx]
        
        # Filter invalid infinity/zeros
        left_valid = [r for r in left_sector if 0.05 < r < OBSTACLE_DIST_THRESH]
        right_valid = [r for r in right_sector if 0.05 < r < OBSTACLE_DIST_THRESH]
        
        turn_cmd = 0.0
        
        # 1. If Obstacle on LEFT -> Turn RIGHT (Negative Z)
        if len(left_valid) > 0:
            # The closer the obstacle, the harder we push away
            min_dist = min(left_valid)
            push = (OBSTACLE_DIST_THRESH - min_dist)  # Stronger push if closer
            turn_cmd -= push * AVOID_WEIGHT

        # 2. If Obstacle on RIGHT -> Turn LEFT (Positive Z)
        if len(right_valid) > 0:
            min_dist = min(right_valid)
            push = (OBSTACLE_DIST_THRESH - min_dist)
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

    # --- PHASE 1.5: SMART CHASE ---
    print(f">>> CHASING TARGET WITH OBSTACLE AVOIDANCE...")
    
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = 'base_link'
    
    while rclpy.ok():
        rclpy.spin_once(listener, timeout_sec=0.1)
        twist_msg.header.stamp = listener.get_clock().now().to_msg()
        
        if not listener.target_detected:
            # Lost logic: Spin slowly to find it? Or Stop?
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0 # Stop for safety
            vel_pub.publish(twist_msg)
            continue
        
        if listener.visual_width >= TARGET_VISUAL_WIDTH:
            print(">>> TARGET REACHED. STOPPING.")
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            vel_pub.publish(twist_msg)
            break
        
        # --- BLENDED CONTROL LAW ---
        # 1. Attraction: Turn towards ball
        vision_turn = listener.track_error * 1.5 
        
        # 2. Repulsion: Turn away from obstacles (calculated in scan_cb)
        avoidance_turn = listener.avoid_turn
        
        # 3. Combine
        final_turn = vision_turn + avoidance_turn
        
        # 4. Safety: If avoidance is huge (very close wall), slow down!
        if abs(avoidance_turn) > 1.0:
            twist_msg.twist.linear.x = 0.05 # Creep
        else:
            twist_msg.twist.linear.x = 0.15 # Normal chase speed
            
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
    wp_pen = create_pose(navigator, 0.0, 3.5, 1.57) 
    
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