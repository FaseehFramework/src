import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

# Settings
msg = """
Control Your Spherebot!
---------------------------
Moving around:
   w
a  s  d
   x

w/x : increase/decrease linear velocity (forward/backward)
a/d : increase/decrease angular velocity (left/right)
s   : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # *** CRITICAL: MATCH THIS TO YOUR CONTROLLER TOPIC ***
        # Usually '/diff_drive_base_controller/cmd_vel_unstamped' for Twist
        # Or '/diff_drive_base_controller/cmd_vel' if you remapped it
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)
        
        self.linear_vel = 0.5
        self.angular_vel = 1.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                twist_stamped = TwistStamped()

                twist_stamped.header.stamp = self.get_clock().now().to_msg()
                twist_stamped.header.frame_id = 'base_link'
                
                if key == 'w':
                    twist_stamped.twist.linear.x = self.linear_vel
                elif key == 'x':
                    twist_stamped.twist.linear.x = -self.linear_vel
                elif key == 'a':
                    twist_stamped.twist.angular.z = self.angular_vel
                elif key == 'd':
                    twist_stamped.twist.angular.z = -self.angular_vel
                elif key == 's':
                    twist_stamped.twist.linear.x = 0.0
                    twist_stamped.twist.angular.z = 0.0
                elif key == '\x03': # CTRL-C
                    break
                else:
                    twist_stamped.twist.linear.x = 0.0
                    twist_stamped.twist.angular.z = 0.0

                self.publisher_.publish(twist_stamped)

        except Exception as e:
            print(e)

        finally:
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_link'
            stop_msg.twist.linear.x = 0.0
            stop_msg.twist.angular.z = 0.0
            self.publisher_.publish(stop_msg)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()