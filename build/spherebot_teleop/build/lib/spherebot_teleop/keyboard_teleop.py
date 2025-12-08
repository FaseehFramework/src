import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray # <--- NEW IMPORT
import sys, select, termios, tty

# Settings
msg = """
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
"""

e = """
Communications Failed
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # 1. Drive Publisher (TwistStamped for Jazzy)
        self.publisher_ = self.create_publisher(
            TwistStamped, 
            '/diff_drive_base_controller/cmd_vel', 
            10
        )

        # 2. Gate Publishers (Float64MultiArray for ForwardCommandController)
        # NOTE: Verify these topic names match "ros2 topic list"
        self.left_gate_pub = self.create_publisher(
            Float64MultiArray, 
            '/left_gate_controller/commands', 
            10
        )
        self.right_gate_pub = self.create_publisher(
            Float64MultiArray, 
            '/right_gate_controller/commands', 
            10
        )
        
        self.linear_vel = 0.5
        self.angular_vel = 1.0
        
        # Gate States (Positions based on your request)
        # Close: Left -0.25, Right 0.25
        # Open:  Left  0.0,  Right 0.0
        self.gate_close_cmd_left = Float64MultiArray(data=[-0.25])
        self.gate_close_cmd_right = Float64MultiArray(data=[0.25])
        
        self.gate_open_cmd_left = Float64MultiArray(data=[0.0])
        self.gate_open_cmd_right = Float64MultiArray(data=[0.0])

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
                
                # --- Drive Logic ---
                twist_stamped = TwistStamped()
                twist_stamped.header.stamp = self.get_clock().now().to_msg()
                twist_stamped.header.frame_id = 'base_link'

                # --- Gate Logic ---
                if key == 'c':
                    print("Gates: Closing...")
                    self.left_gate_pub.publish(self.gate_close_cmd_left)
                    self.right_gate_pub.publish(self.gate_close_cmd_right)
                
                elif key == 'o':
                    print("Gates: Opening...")
                    self.left_gate_pub.publish(self.gate_open_cmd_left)
                    self.right_gate_pub.publish(self.gate_open_cmd_right)

                # --- Drive Keys ---
                elif key == 'w':
                    twist_stamped.twist.linear.x = self.linear_vel
                    self.publisher_.publish(twist_stamped)
                elif key == 'x':
                    twist_stamped.twist.linear.x = -self.linear_vel
                    self.publisher_.publish(twist_stamped)
                elif key == 'a':
                    twist_stamped.twist.angular.z = self.angular_vel
                    self.publisher_.publish(twist_stamped)
                elif key == 'd':
                    twist_stamped.twist.angular.z = -self.angular_vel
                    self.publisher_.publish(twist_stamped)
                elif key == 's':
                    twist_stamped.twist.linear.x = 0.0
                    twist_stamped.twist.angular.z = 0.0
                    self.publisher_.publish(twist_stamped)
                elif key == '\x03': # CTRL-C
                    break
                else:
                    # Optional: Stop on key release behavior if desired
                    pass

        except Exception as e:
            print(e)

        finally:
            # Stop command on exit
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_link'
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