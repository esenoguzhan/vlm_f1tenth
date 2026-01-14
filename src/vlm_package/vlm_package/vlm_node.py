import rclpy
import re
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from PIL import Image as im

# Specific to Jetson AI Lab / nano_llm environment
from nano_llm import NanoLLM, ChatHistory 

class VLMNode(Node):
    def __init__(self):
        super().__init__('vlm_node')

        # --- 1. DECLARE PARAMETERS ---
        self.declare_parameter('model_path', '/home/user/models/vila-1.5-3b')
        self.declare_parameter('api', 'mlc')
        self.declare_parameter('quantization', 'q4f16_ft')
        self.declare_parameter('min_new_tokens', 25)
        self.declare_parameter('do_sample', False)
        self.declare_parameter('history_depth', 1) # Calibratable depth
        self.declare_parameter('default_query', '')

        # --- 2. MODEL LOADING ---
        self.get_logger().info("Loading VILA 1.5-3b into GPU...")
        self.model = NanoLLM.from_pretrained(
            self.get_parameter('model_path').value, 
            api=self.get_parameter('api').value, 
            quantization=self.get_parameter('quantization').value
        )
        self.chat_history = ChatHistory(self.model)
        self.cv_br = CvBridge()

        # --- 3. SYNCHRONIZED SUBSCRIPTIONS ---
        # Remap these in your launch file to: /zed/zed_node/left/image_rect_color and /pose
        self.image_sub = Subscriber(self, Image, 'input_image')
        self.pose_sub = Subscriber(self, PoseStamped, 'slam_pose')
        
        # Syncs camera and pose within 0.1s of each other
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.pose_sub], 10, 0.1)
        self.ts.registerCallback(self.synchronized_callback)

        # --- 4. PUBLISHERS ---
        self.waypoint_pub = self.create_publisher(PointStamped, 'vlm/target_waypoint', 10)
        self.command_pub = self.create_publisher(String, 'vlm/high_level_command', 10)
        
        self.get_logger().info("VLM Node is fully operational and closed-loop ready.")

    def synchronized_callback(self, img_msg, pose_msg):
        # --- ROLLING WINDOW MEMORY MANAGEMENT ---
        depth = self.get_parameter('history_depth').value
        if depth <= 1:
            self.chat_history.reset() # Purely reactive mode
        else:
            # nano_llm adds ~3 entries per turn (img, prompt, output)
            # Maintain a window by removing the oldest turn
            while len(self.chat_history) > (depth * 3 - 3):
                self.chat_history.remove(0)

        # --- DATA PREPARATION ---
        cv_img = self.cv_br.imgmsg_to_cv2(img_msg, 'rgb8')
        pil_img = im.fromarray(cv_img)
        
        # Append spatial context to prompt
        query = self.get_parameter('default_query').value
        prompt = f"Current Pose: ({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f}). {query}"

        # --- VILA INFERENCE ---
        self.chat_history.append('user', image=pil_img)
        self.chat_history.append('user', prompt)
        embedding, _ = self.chat_history.embed_chat()

        output = self.model.generate(
            inputs=embedding,
            kv_cache=self.chat_history.kv_cache,
            min_new_tokens=self.get_parameter('min_new_tokens').value,
            do_sample=self.get_parameter('do_sample').value
        )
        
        # Publish using image timestamp for control latency compensation
        self.parse_and_publish(output, img_msg.header.stamp)

    def parse_and_publish(self, text, stamp):
        try:
            # Robust Greedy Parsing
            clean_text = text.replace('`', '').strip()
            
            # 1. Coordinate Extraction: [x, y]
            coord_match = re.search(r"\[\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\]", clean_text)
            
            # 2. Angle Extraction: number following the comma after coordinates
            angle_match = re.search(r"\]\s*,\s*(-?\d+\.?\d*)", clean_text)
            
            # 3. Behavioral Command Extraction
            command_match = re.search(r"(fast|slow|stop)", clean_text.lower())

            if coord_match and angle_match and command_match:
                x, y = map(float, coord_match.groups())
                angle = float(angle_match.group(1))
                command = command_match.group(0)

                # Target Waypoint (Published in Car Frame 'base_link')
                wp = PointStamped()
                wp.header.stamp = stamp
                wp.header.frame_id = "base_link" 
                wp.point.x, wp.point.y = x, y
                self.waypoint_pub.publish(wp)

                # Behavioral Command + Angle for Stanley Controller
                cmd_msg = String()
                cmd_msg.data = f"{command}, angle:{angle}"
                self.command_pub.publish(cmd_msg)
                
        except Exception as e:
            self.get_logger().error(f"Robust Parsing Failed on text: '{text}'. Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()