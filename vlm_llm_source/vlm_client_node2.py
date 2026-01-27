#!/usr/bin/env python3
"""
ROS 2 VLM Client Node
Sends camera images to Mac VLM server and receives driving decisions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import socket
import struct
import json
import threading
import numpy as np


class VLMClientNode(Node):
    def __init__(self):
        super().__init__("vlm_client_node")

        # Parameters
        self.declare_parameter("server_ip", "192.168.1.100")  # Mac IP address
        self.declare_parameter("server_port", 9999)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("use_compressed", False)
        self.declare_parameter("send_interval", 1.0)  # Send interval (seconds)

        self.server_ip = self.get_parameter("server_ip").value
        self.server_port = self.get_parameter("server_port").value
        self.image_topic = self.get_parameter("image_topic").value
        self.use_compressed = self.get_parameter("use_compressed").value
        self.send_interval = self.get_parameter("send_interval").value

        self.bridge = CvBridge()
        self.latest_image = None
        self.lock = threading.Lock()
        self.socket = None
        self.connected = False

        # Subscribe to image topic
        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.image_topic + "/compressed",
                self.compressed_image_callback,
                10
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                10
            )

        # Publish decision
        self.decision_pub = self.create_publisher(String, "/vlm_decision", 10)

        # Timer for sending images
        self.timer = self.create_timer(self.send_interval, self.send_to_vlm)

        # Connect to server
        self.connect_to_server()

        self.get_logger().info(f"VLM client started, server: {self.server_ip}:{self.server_port}")

    def connect_to_server(self):
        """Connect to Mac VLM server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_ip, self.server_port))
            self.connected = True
            self.get_logger().info("Connected to VLM server")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to server: {e}")
            self.connected = False

    def image_callback(self, msg: Image):
        """Process raw image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def compressed_image_callback(self, msg: CompressedImage):
        """Process compressed image"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self.lock:
                self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Compressed image decode failed: {e}")

    def send_to_vlm(self):
        """Send image to VLM server"""
        if not self.connected:
            self.connect_to_server()
            return

        with self.lock:
            if self.latest_image is None:
                return
            image = self.latest_image.copy()

        try:
            # Compress image to JPEG
            _, jpeg_data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            image_bytes = jpeg_data.tobytes()

            # Send: length + data
            self.socket.sendall(struct.pack(">I", len(image_bytes)))
            self.socket.sendall(image_bytes)

            # Receive decision
            length_data = self.socket.recv(4)
            if len(length_data) < 4:
                raise ConnectionError("Connection closed")

            msg_length = struct.unpack(">I", length_data)[0]
            response_data = b""
            while len(response_data) < msg_length:
                chunk = self.socket.recv(msg_length - len(response_data))
                if not chunk:
                    raise ConnectionError("Connection closed")
                response_data += chunk

            # Parse decision from vlm_server_mlx.py
            decision = json.loads(response_data.decode("utf-8"))
            decision_str = decision.get('decision', 'unknown')
            inference_time = decision.get('inference_time', 0.0)
            success = decision.get('success', True)

            self.get_logger().info(
                f"Decision: {decision_str} | inference_time: {inference_time:.2f}s | success: {success}"
            )

            # Publish decision
            msg = String()
            msg.data = json.dumps(decision)
            self.decision_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Communication error: {e}")
            self.connected = False
            if self.socket:
                self.socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = VLMClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
