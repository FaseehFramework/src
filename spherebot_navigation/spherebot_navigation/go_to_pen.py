#!/usr/bin/env python3
"""
Simple Waypoint Follower to Pen Goal
Navigates the robot to the pen goal coordinates using Nav2.
"""

import math
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


def create_pose(navigator, x, y, yaw_rad):
    """Create a PoseStamped message for navigation."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw_rad / 2)
    pose.pose.orientation.w = math.cos(yaw_rad / 2)
    return pose


def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create the navigator
    navigator = BasicNavigator()
    
    # Set initial pose (adjust these coordinates based on your robot's starting position)
    initial_x = -3.69
    initial_y = 0.0
    initial_yaw = 0.0
    
    initial_pose = create_pose(navigator, initial_x, initial_y, initial_yaw)
    navigator.setInitialPose(initial_pose)
    
    # Wait for Nav2 to be active
    print(">>> Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print(">>> Nav2 is active!")
    
    # Define the pen goal coordinates
    pen_x = 0.03
    pen_y = 2.5
    pen_yaw = 1.57  # radians (approximately 90 degrees)
    
    wp_pen = create_pose(navigator, pen_x, pen_y, pen_yaw)
    
    # Navigate to the pen goal
    print(f">>> Navigating to pen goal: ({pen_x}, {pen_y}, {pen_yaw} rad)...")
    navigator.goToPose(wp_pen)
    
    # Monitor navigation progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"    Distance remaining: {feedback.distance_remaining:.2f} m")
        rclpy.spin_once(navigator, timeout_sec=0.1)
    
    # Check the result
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print(">>> SUCCESS! Robot reached the pen goal!")
    elif result == TaskResult.CANCELED:
        print(">>> CANCELED: Navigation was canceled.")
    elif result == TaskResult.FAILED:
        print(">>> FAILED: Could not reach the pen goal.")
        print("    Check if the goal is reachable and within the map boundaries.")
    else:
        print(f">>> UNKNOWN RESULT: {result}")
    
    # Shutdown
    rclpy.shutdown()


if __name__ == '__main__':
    main()
