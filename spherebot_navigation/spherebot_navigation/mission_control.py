import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# CHANGE 1: Import TwistStamped instead of Twist
from geometry_msgs.msg import PoseStamped, TwistStamped, Point 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import JointState

# --- CONFIGURATION ---
INITIAL_X = -3.69
INITIAL_Y = 0.0
INITIAL_YAW = 0.0

TARGET_VISUAL_WIDTH = 0.20 

class MissionListener(Node):
    def __init__(self):
        super().__init__('mission_listener')
        self.track_error = 0.0
        self.visual_width = 0.0
        self.target_detected = False
        self.gates_closed = False
        
        self.create_subscription(Point, '/vision/tracking_data', self.vision_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)

    def vision_cb(self, msg):
        if msg.z == 1.0:
            self.target_detected = True
            self.track_error = msg.x 
            self.visual_width = msg.y 
        else:
            self.target_detected = False

    def joint_cb(self, msg):
        try:
            if 'left_gate_joint' in msg.name:
                left_idx = msg.name.index('left_gate_joint')
                right_idx = msg.name.index('right_gate_joint')
                if abs(msg.position[left_idx]) > 0.5 and abs(msg.position[right_idx]) > 0.5:
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
    
    # CHANGE 2: Publisher must use TwistStamped
    vel_pub = listener.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)

    # 1. Init Pose
    initial_pose = create_pose(navigator, INITIAL_X, INITIAL_Y, INITIAL_YAW)
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    # 2. Waypoints
    wp_sphere = create_pose(navigator, 1.5, 0.0, 0.0)
    wp_pen = create_pose(navigator, 7.0, 34.0, 58.0)

    # ==========================
    # PHASE 1: SEARCH
    # ==========================
    print(">>> PHASE 1: Moving to Search Area...")
    navigator.goToPose(wp_sphere)

    while not navigator.isTaskComplete():
        rclpy.spin_once(listener, timeout_sec=0.1)
        
        if listener.target_detected:
            print(f"\n!!! TARGET SPOTTED (Width: {listener.visual_width:.2f}) !!!")
            print("!!! SWITCHING TO VISUAL APPROACH !!!")
            navigator.cancelTask()
            break

    # ==========================
    # PHASE 1.5: VISUAL APPROACH
    # ==========================
    print(f">>> APPROACHING TARGET (Stop Threshold: {TARGET_VISUAL_WIDTH})...")
    
    # CHANGE 3: Create TwistStamped object
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = 'base_link'
    
    while rclpy.ok():
        rclpy.spin_once(listener, timeout_sec=0.1)
        
        # Update timestamp every loop
        twist_msg.header.stamp = listener.get_clock().now().to_msg()
        
        # 1. LOST TARGET CHECK
        if not listener.target_detected:
            print(">>> LOST TARGET! STOPPING.")
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            vel_pub.publish(twist_msg)
            break 
        
        # 2. STOP CONDITION (Too Close)
        if listener.visual_width >= TARGET_VISUAL_WIDTH:
            print(f">>> STOPPING. Visual Width {listener.visual_width:.2f} >= {TARGET_VISUAL_WIDTH}")
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            vel_pub.publish(twist_msg)
            break
        
        # 3. DRIVE LOGIC
        twist_msg.twist.linear.x = 0.15 
        twist_msg.twist.angular.z = listener.track_error * 1.5 
        
        vel_pub.publish(twist_msg)

    # ==========================
    # PHASE 2: MANUAL DOCKING
    # ==========================
    print("\n>>> PHASE 2: MANUAL CONTROL ENGAGED.")
    print(">>> USE TELEOP TO CAPTURE. CLOSE GATES ('c') TO FINISH.")
    
    while not listener.gates_closed:
        rclpy.spin_once(listener, timeout_sec=0.1)
        time.sleep(0.1)

    print(">>> GATES CLOSED. RETURNING HOME.")
    time.sleep(1.0)

    # ==========================
    # PHASE 3: RETURN
    # ==========================
    navigator.goToPose(wp_pen)
    while not navigator.isTaskComplete():
        rclpy.spin_once(listener, timeout_sec=0.1)

    print(">>> MISSION COMPLETE.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()