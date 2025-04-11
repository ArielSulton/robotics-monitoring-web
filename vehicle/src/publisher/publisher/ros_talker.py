#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import random

class Talker(Node):
    def __init__(self):
        super().__init__('talker_node')

        self.publisher_image = self.create_publisher(Image, '/vehicle/camera', 10)
        self.publisher_rpy = self.create_publisher(Float32MultiArray, '/vehicle/imu', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the camera.")
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame.")
            return
        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_image.publish(msg)
        
        rpy = Float32MultiArray()
        rpy.data = [
            round(random.uniform(-180, 180), 2),  # Roll
            round(random.uniform(-90, 90), 2),    # Pitch
            round(random.uniform(-180, 180), 2)   # Yaw
        ]
        self.publisher_rpy.publish(rpy)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()