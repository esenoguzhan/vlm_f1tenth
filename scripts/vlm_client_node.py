#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import cv2
import socket
import struct
import json
import threading
import numpy as np


class VLMClientNode(Node):
    def __init__(self):
        super().__init__("vlm_client_node")

        #paramaters
        self.declare_parameter("server_ip", "10.183.230.216")  # Mac IP
        self.declare_parameter("server_port", 9999)
        self.declare_parameter("image_topic", "/zed/zed_node/right/image_rect_color")
        self.declare_parameter("use_compressed", False)
        self.declare_parameter("send_interval", 1.0)  

        self.server_ip = self.get_parameter("server_ip").value
        self.server_port = self.get_parameter("server_port").value
        self.image_topic = self.get_parameter("image_topic").value
        self.use_compressed = self.get_parameter("use_compressed").value
        self.send_interval = self.get_parameter("send_interval").value

        self.latest_image = None
        self.lock = threading.Lock()
        self.socket = None
        self.connected = False

        # image topic sub
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

        # pub image
        self.decision_pub = self.create_publisher(String, "/vlm_overtake", 10)

        # set timer
        self.timer = self.create_timer(self.send_interval, self.send_to_vlm)

        # 连接服务器
        self.connect_to_server()

        self.get_logger().info(f"VLM server runing，server: {self.server_ip}:{self.server_port}")

    def connect_to_server(self):
        """connected Mac VLM server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_ip, self.server_port))
            self.connected = True
            self.get_logger().info("already connected to VLM server")
        except Exception as e:
            self.get_logger().error(f"connect to server failed: {e}")
            self.connected = False

    def image_callback(self, msg: Image):
        """processing the raw image"""
        try:
        
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            
            if msg.encoding == "rgb8":
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            with self.lock:
                self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"process image failed: {e}")

    def compressed_image_callback(self, msg: CompressedImage):
        """pocess compress image failed"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self.lock:
                self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"encode failed: {e}")

    def send_to_vlm(self):
        """send image to mac sever"""
        if not self.connected:
            self.connect_to_server()
            return

        with self.lock:
            if self.latest_image is None:
                return
            image = self.latest_image.copy()

        try:
            # compress image to JPEG
            _, jpeg_data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            image_bytes = jpeg_data.tobytes()

            # send
            self.socket.sendall(struct.pack(">I", len(image_bytes)))
            self.socket.sendall(image_bytes)

            # receive decision
            length_data = self.socket.recv(4)
            if len(length_data) < 4:
                raise ConnectionError("connect broken")

            msg_length = struct.unpack(">I", length_data)[0]
            response_data = b""
            while len(response_data) < msg_length:
                chunk = self.socket.recv(msg_length - len(response_data))
                if not chunk:
                    raise ConnectionError("connect broken")
                response_data += chunk

            # decision
            decision = json.loads(response_data.decode("utf-8"))
            self.get_logger().info(
                f"decision: {decision['decision']} "
                #f"(confidence: {decision['confidence']:.2f}) - {decision['reason']}"
            )

            # decision pub
            msg = String()
            msg.data = json.dumps(decision)
            self.decision_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"comunication failed: {e}")
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
