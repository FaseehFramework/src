import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point # Using Point to send (Steering, Distance, Flag)
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        
        # Publishing geometry_msgs/Point
        # x = Steering Error (-1.0 left to 1.0 right)
        # y = Proximity/Width (0.0 far to 1.0 close)
        # z = Detection Flag (1.0 = found, 0.0 = lost)
        self.publisher_ = self.create_publisher(Point, '/vision/tracking_data', 10)
        
        self.br = CvBridge()

    def image_callback(self, data):
        try:
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except Exception:
            return

        h, w_img, _ = current_frame.shape
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_contour = None
        max_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100: # Filter noise
                if area > max_area:
                    max_area = area
                    largest_contour = contour

        msg = Point()
        
        if largest_contour is not None:
            # 1. Calculate Bounding Box
            x, y, w_box, h_box = cv2.boundingRect(largest_contour)
            
            # 2. Calculate Centroid for Steering
            center_x = x + (w_box / 2)
            steering_error = -1.0 * (center_x - (w_img / 2)) / (w_img / 2)
            
            # 3. Calculate "Closeness" (Box Width / Image Width)
            # Far away ball might be 0.05 (5% of screen)
            # Close ball might be 0.40 (40% of screen)
            proximity = float(w_box) / float(w_img)
            
            msg.x = float(steering_error)
            msg.y = float(proximity)
            msg.z = 1.0 # Detection Flag: True
            
        else:
            msg.z = 0.0 # Detection Flag: False

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()