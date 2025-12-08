#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomFixer(Node):
    def __init__(self):
        super().__init__('odom_fixer')
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_base_controller/odom',
            self.odom_callback,
            10)
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/diff_drive_base_controller/odom_fixed',
            10)
            
        self.get_logger().info('Odom Fixer Node Started')

    def odom_callback(self, msg):
        # Fix the frame IDs
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        # Republish
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
