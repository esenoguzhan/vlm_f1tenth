import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading
import socket
import time
import traceback

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
            print("Server socket bound and listening.")
            
            # This blocks until connection
            self.client_socket, addr = self.server_socket.accept()
            print(f"Microphone connected from: {addr}")
        except Exception as e:
            print(f"Network Mic Bind/Accept Error: {e}")
            raise e
        return self

    def __exit__(self, exc_type, exc_value, tb):
        print("Closing Network Microphone socket...")
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
                        # socket closed by client
                        # print("Socket EOF encountered.")
                        return b""  # EOF
                    data += packet
            except Exception as e:
                print(f"Socket read error: {e}")
                return b""
        return data

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        self.recognizer = sr.Recognizer()
        
        # Start the listening thread
        self.listen_thread = threading.Thread(target=self.start_listening)
        self.listen_thread.daemon = True # Kill thread if main process dies
        self.listen_thread.start()
        
    def start_listening(self):
        self.get_logger().info('Starting Network Microphone Server...')
        
        while rclpy.ok():
            try:
                # Re-create the network mic source for each new connection
                with NetworkMicrophone(port=9000) as source:
                    self.get_logger().info('Client connected. Listening for speech...')
                    
                    try:
                        while rclpy.ok():
                            # This blocks until a phrase is detected
                            # self.get_logger().info('Waiting for speech input...')
                            audio = self.recognizer.listen(source, phrase_time_limit=10)
                            if len(audio.frame_data) == 0:
                                self.get_logger().warn("Empty audio received, client likely disconnected.")
                                break
                                
                            self.process_audio(audio)
                            
                    except Exception as e:
                        self.get_logger().warn(f"Processing loop error: {e}")
                        # traceback.print_exc()
                        break 
                        
            except Exception as e:
                self.get_logger().error(f"Server bind/accept error: {e}")
                time.sleep(2) # Wait before retry

    def process_audio(self, audio):
        try:
            self.get_logger().info(f'Processing {len(audio.frame_data)} bytes of audio...')
            text = self.recognizer.recognize_google(audio)
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Recognized: "{text}"')
        except sr.UnknownValueError:
            self.get_logger().warn('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Service error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in recognition: {e}')

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
