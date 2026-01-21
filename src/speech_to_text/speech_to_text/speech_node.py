import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading
import socket
import time

class NetworkMicrophone(sr.AudioSource):
    """Custom AudioSource that reads from a TCP socket"""
    def __init__(self, host='0.0.0.0', port=9000, sample_rate=16000):
        self.host = host
        self.port = port
        self.SAMPLE_RATE = sample_rate
        self.SAMPLE_WIDTH = 2  # 16-bit
        self.CHUNK = 1024
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket = None

    def __enter__(self):
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            print(f"Network Mic listening on {self.host}:{self.port}...")
            print("Please run 'python stream_mic.py' on your Windows PC now.")
            self.client_socket, addr = self.server_socket.accept()
            print(f"Microphone connected from: {addr}")
        except Exception as e:
            print(f"Network Mic Bind Error: {e}")
            raise e
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()

    @property
    def stream(self):
        return self

    def read(self, size):
        """Read exact size bytes from socket"""
        data = b""
        if self.client_socket:
            try:
                # Loop to ensure we get exactly 'size' bytes or until EOF
                while len(data) < size:
                    packet = self.client_socket.recv(size - len(data))
                    if not packet:
                        return b""  # EOF
                    data += packet
            except Exception:
                return b""
        return data

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        self.recognizer = sr.Recognizer()
        
        # Start the listening thread
        self.listen_thread = threading.Thread(target=self.start_listening)
        self.listen_thread.start()
        
    def start_listening(self):
        self.get_logger().info('Starting Network Microphone Server...')
        
        # We wrap the network source in a loop to allow reconnection
        while rclpy.ok():
            try:
                with NetworkMicrophone(port=9000) as source:
                    self.get_logger().info('Client connected. Listening for speech...')
                    
                    try:
                        while rclpy.ok():
                            # We manually listen in a loop to handle the stream
                            # This blocks until a phrase is detected
                            audio = self.recognizer.listen(source, phrase_time_limit=10)
                            self.process_audio(audio)
                    except Exception as e:
                        self.get_logger().warn(f"Connection lost or error: {e}")
                        
            except Exception as e:
                self.get_logger().error(f"Server error: {e}")
                time.sleep(2) # Wait before restart

    def process_audio(self, audio):
        try:
            self.get_logger().info('Processing audio...')
            text = self.recognizer.recognize_google(audio)
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Recognized: "{text}"')
        except sr.UnknownValueError:
            self.get_logger().warn('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Service error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
