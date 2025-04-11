#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import threading

from flask import Flask, Response, render_template, jsonify
import time

# Global variables
data_lock = threading.Lock()
listener_instance = None
app = Flask(__name__)

# ROS2 Node
class Listener(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/vehicle/camera', self.image_callback, 10)
        self.create_subscription(Float32MultiArray, '/vehicle/imu', self.rpy_callback, 10)

        self.image_viz = None
        self.image_event = threading.Event()
        self.latest_rpy = [0.0, 0.0, 0.0]

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with data_lock:
            self.image_viz = cv2.imencode(".jpg", cv_image)[1].tobytes()
            self.image_event.set()

    def get_image(self):
        self.image_event.wait()
        with data_lock:
            image_data = self.image_viz
            self.image_event.clear()
        return image_data

    def rpy_callback(self, msg):
        with data_lock:
            self.latest_rpy = list(msg.data)

# Flask Routes
@app.route('/')
def index():
    return render_template("index.html")

def generate_frames():
    while True:
        if listener_instance is None:
            continue
        image_data = listener_instance.get_image()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + image_data + b'\r\n')

@app.route('/image_response')
def image_response():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/orientation')
def get_orientation():
    if listener_instance is None:
        return jsonify({'error': 'ROS listener not ready'}), 503
    with data_lock:
        rpy_copy = listener_instance.latest_rpy.copy()
    return jsonify({
        'roll': round(rpy_copy[0], 2),
        'pitch': round(rpy_copy[1], 2),
        'yaw': round(rpy_copy[2], 2)
    })

# ROS Node Thread
def ros_thread():
    global listener_instance
    rclpy.init()
    listener_instance = Listener()
    try:
        rclpy.spin(listener_instance)
    finally:
        listener_instance.destroy_node()
        rclpy.shutdown()

# Main
if __name__ == '__main__':
    threading.Thread(target=ros_thread, daemon=True).start()

    while listener_instance is None:
        print("Waiting for ROS node to initialize...")
        time.sleep(0.5)

    app.run(host='0.0.0.0', port=5000, debug=True)