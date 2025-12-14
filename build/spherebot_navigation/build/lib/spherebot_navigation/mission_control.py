import time
from copy import deepcopy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

"""
Mission Control Script
----------------------
1. Initialize Navigator
2. Set Initial Pose (Estimate where robot is)
3. Go to Red Sphere (Waypoint 1)
4. Go to Blue Sphere (Waypoint 2)
5. Return to Pen/Start (Waypoint 3)
"""

def main():
    rclpy.init()

    # 1. Initialize the Navigator
    navigator = BasicNavigator()

    # 2. Set Initial Pose
    # This simulates clicking "2D Pose Estimate" in RViz.
    # We assume the robot starts at (0, 0) facing East (0.0).
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -3.69
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    
    # Send the initial pose to AMCL
    navigator.setInitialPose(initial_pose)

    # 3. Wait for Nav2 to fully start
    # This waits for the "autostart" nodes to be active
    navigator.waitUntilNav2Active()

    # --- DEFINE WAYPOINTS ---
    # Coordinates based on your assessment_world.sdf
    
    # Waypoint 1: Near Red Sphere (x=2.0, y=0.0)
    # We stop slightly before it to avoid collision
    wp1 = PoseStamped()
    wp1.header.frame_id = 'map'
    wp1.header.stamp = navigator.get_clock().now().to_msg()
    wp1.pose.position.x = 1.5 
    wp1.pose.position.y = 0.0
    wp1.pose.orientation.w = 1.0

    # Waypoint 2: Near Blue Sphere (x=2.0, y=1.0)
    wp2 = deepcopy(wp1)
    wp2.pose.position.x = 1.5
    wp2.pose.position.y = 1.0

    # Waypoint 3: Return to Pen (Start)
    wp3 = deepcopy(wp1)
    wp3.pose.position.x = 0.0
    wp3.pose.position.y = 0.0


    # --- EXECUTE MISSION ---
    waypoints = [wp1, wp2, wp3]

    # Option A: Go to each point sequentially (Allows logic in between)
    for i, wp in enumerate(waypoints):
        print(f"Moving to Waypoint {i+1}...")
        
        navigator.goToPose(wp)

        while not navigator.isTaskComplete():
            # Process feedback (optional)
            feedback = navigator.getFeedback()
            # print(f'Distance remaining: {feedback.distance_remaining:.2f} meters')

        # Check Result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Waypoint {i+1} Reached!")
            # Placeholder: Insert Gate Logic Here Later
            # e.g., close_gates()
            time.sleep(2.0) # Pause for effect
        elif result == TaskResult.CANCELED:
            print(f"Waypoint {i+1} Canceled!")
            exit(1)
        elif result == TaskResult.FAILED:
            print(f"Waypoint {i+1} Failed!")
            exit(1)

    print("Mission Complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()