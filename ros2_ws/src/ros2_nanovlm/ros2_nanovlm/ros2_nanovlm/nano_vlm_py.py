import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from PIL import Image as im
from nano_llm import NanoLLM, ChatHistory
import numpy as np
import threading

 

class Nano_LLM_Subscriber(Node):

    def __init__(self):
        super().__init__('nano_llm_subscriber')
        
        #EDIT MODEL HERE
        self.declare_parameter('model', "Efficient-Large-Model/VILA1.5-3b")
        self.declare_parameter('api', "mlc")
        self.declare_parameter('quantization', "q4f16_ft")
        self.declare_parameter('max_context_len', "1024")
        self.declare_parameter('image_size', 224)      # Larger = more accurate
        self.declare_parameter('frame_skip', 3)        # Process every Nth frame

        # Frame counter for skipping
        self.frame_count = 0
        self.last_decision = 'NO_CAR'

        # Threading control
        self.is_processing = False
        self.processing_lock = threading.Lock()
      
        # Subscriber for input query
        self.query_subscription = self.create_subscription(
            String,
            'input_query',
            self.query_listener_callback,
            10)
        self.query_subscription  # prevent unused variable warning

        # Subscriber for input image
        self.image_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/right_raw/image_raw_color',
            self.image_listener_callback,
            10)
        self.image_subscription  # prevent unused variable warning

        # To convert ROS image message to OpenCV image
        self.cv_br = CvBridge() 
      
        #load the model 
        self.model = NanoLLM.from_pretrained(
        model = "Efficient-Large-Model/VILA1.5-3b", 
        max_context_len = 1024
        )

        #chatHistory var 
        self.chat_history = ChatHistory(self.model)

        ##  PUBLISHER
        self.output_publisher = self.create_publisher(String, 'output', 10)
     
        #self.query = "Describe where the car is in this image. Say LEFT, CENTER, RIGHT, or NONE."
        self.query = "Is the orange traffic cone on the left or the right?"

        # Timer-based publishing at fixed rate (20 Hz)
        self.declare_parameter('publish_rate', 20.0)
        pub_rate = self.get_parameter('publish_rate').value
        self.publish_timer = self.create_timer(1.0 / pub_rate, self.publish_decision)

    def query_listener_callback(self, msg):
        self.query = msg.data

    def publish_decision(self):
        """Timer callback - publish last decision at fixed rate"""
        output_msg = String()
        output_msg.data = self.last_decision
        self.output_publisher.publish(output_msg)


    def image_listener_callback(self, data):
        # Frame skipping - only process every Nth frame
        self.frame_count += 1
        frame_skip = self.get_parameter('frame_skip').value
        if self.frame_count % frame_skip != 0:
            return  # Timer handles publishing

        # Skip if already processing
        if self.is_processing:
            return

        # Run inference in background thread
        threading.Thread(target=self.run_inference, args=(data,)).start()

    def run_inference(self, data):
        with self.processing_lock:
            self.is_processing = True
            try:
                prompt = self.query

                # Convert ROS image to PIL with resize for speed
                cv_img = self.cv_br.imgmsg_to_cv2(data, 'rgb8')
                img_size = self.get_parameter('image_size').value
                cv_img = cv2.resize(cv_img, (img_size, img_size))
                PIL_img = im.fromarray(cv_img)

                # Build chat history with image and prompt
                self.chat_history.append('user', image=PIL_img)
                self.chat_history.append('user', prompt)
                embedding, _ = self.chat_history.embed_chat()

                output = self.model.generate(
                    inputs=embedding,
                    kv_cache=self.chat_history.kv_cache,
                    max_new_tokens=25,
                    min_new_tokens=5,
                    streaming=False,
                    do_sample=True,
                    temperature=0.3,
                    top_p=0.98
                )

                # Log original model output before any processing
                self.get_logger().info(f"Original LLM output: {repr(output)}")

                # Clean output: remove special tokens
                raw_output = str(output).replace('</s>', '').replace('<s>', '').strip().upper()

                # Priority: position words first (LEFT/CENTER/RIGHT/NONE/NO)
                position_words = {
                    'LEFT': 'CAR_LEFT',
                    'CENTER': 'CAR_CENTER',
                    'RIGHT': 'CAR_RIGHT',
                }

                clean_output = 'NO_CAR'  # Default fallback
                words = raw_output.replace(',', ' ').replace('.', ' ').split()

                # First look for position words
                for word in words:
                    if word in position_words:
                        clean_output = position_words[word]
                        break

                self.get_logger().info(f"Raw: {raw_output} -> Parsed: {clean_output}")

                # Save decision (timer handles publishing)
                self.last_decision = clean_output
                self.get_logger().info(f"VLM Decision: {clean_output}")
                

                self.chat_history.reset()
            except Exception as e:
                self.get_logger().error(f"Inference error: {e}")
            finally:
                self.is_processing = False



def main(args=None):
    rclpy.init(args=args)

    nano_llm_subscriber = Nano_LLM_Subscriber()

    # Use MultiThreadedExecutor so timer runs while inference is happening
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nano_llm_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nano_llm_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

