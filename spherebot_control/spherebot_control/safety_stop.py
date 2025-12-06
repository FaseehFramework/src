#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')
        
        # Parameters
        self.stop_distance = 0.5  # meters
        self.scan_topic = '/scan'
        self.cmd_vel_in_topic = '/cmd_vel'
        self.cmd_vel_out_topic = '/diff_drive_base_controller/cmd_vel_unstamped'

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_in_topic,
            self.cmd_vel_callback,
            10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_out_topic,
            10)

        self.obstacle_detected = False
        self.get_logger().info('Safety Stop Node Started')

    def scan_callback(self, msg):
        # Check for obstacles in front (assuming 0 degrees is front)
        # We check a small arc around the front (-30 to +30 degrees)
        # LaserScan ranges are usually from -PI to +PI or 0 to 2PI.
        # Based on URDF: min_angle=-3.14, max_angle=3.14. So 0 is front.
        
        # Find indices corresponding to -30 to +30 degrees
        # angle_min = -3.14, angle_increment = resolution (1 degree = 0.017 rad)
        # We need to calculate indices carefully.
        
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        # Define arc
        arc_angle = 0.52 # ~30 degrees
        
        # Indices
        # Front is at index corresponding to angle 0.
        # 0 = angle_min + i * angle_inc => i = -angle_min / angle_inc
        center_index = int(-angle_min / angle_inc)
        steps = int(arc_angle / angle_inc)
        
        start_index = max(0, center_index - steps)
        end_index = min(len(msg.ranges), center_index + steps)
        
        # Check ranges
        min_dist = float('inf')
        for i in range(start_index, end_index):
            r = msg.ranges[i]
            if r < msg.range_min or r > msg.range_max:
                continue
            if r < min_dist:
                min_dist = r
                
        if min_dist < self.stop_distance:
            if not self.obstacle_detected:
                self.get_logger().warn(f'Obstacle detected at {min_dist:.2f}m! Stopping.')
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def cmd_vel_callback(self, msg):
        # If obstacle detected and trying to move forward (linear.x > 0), stop.
        if self.obstacle_detected and msg.linear.x > 0:
            msg.linear.x = 0.0
            # Allow rotation? Yes, usually safe to turn in place.
        
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
