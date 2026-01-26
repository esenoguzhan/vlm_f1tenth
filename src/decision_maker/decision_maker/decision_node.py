import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String, Int32
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        # Subscribe to LLM output
        self.subscription = self.create_subscription(
            String,
            'llm_vel',
            self.mode_callback,
            10)
        
        # Publisher for Lane Changing
        self.lane_publisher = self.create_publisher(Int32, '/target_lane', 10)
        
        # Client to set parameters on the controller node
        self.param_client = self.create_client(SetParameters, '/stanley_controller/set_parameters')
        
        self.current_mode = "NORMAL"
        self.get_logger().info('Decision Node started. Waiting for driving mode from LLM...')

        # Subscribe to VLM output for overtaking
        self.vlm_subscription = self.create_subscription(
            String,
            '/vlm_overtake',
            self.vlm_callback,
            10)

    def mode_callback(self, msg):
        try:
            data = json.loads(msg.data)
            raw_text = data.get("action", "").upper()
            self.get_logger().info(f"Received JSON action: {raw_text} (Reason: {data.get('reason', 'N/A')})")
        except json.JSONDecodeError:
            self.get_logger().warn("Received non-JSON input, falling back to raw string")
            raw_text = msg.data.upper()

        detected_mode = None

        # 1. Lane Change Logic
        if "GO_LEFT" in raw_text:
            self.change_lane(1) # 1 is Left
        elif "GO_RIGHT" in raw_text:
            self.change_lane(0) # 0 is Right

        # 2. Driving Mode Logic
        if "DECELERATE" in raw_text:
            detected_mode = "SAFETY"
        elif "ACCELERATE" in raw_text:
            detected_mode = "AGGRESSIVE"
        elif "NORMAL" in raw_text:
            detected_mode = "NORMAL"

        if detected_mode:
            if detected_mode != self.current_mode:
                self.get_logger().info(f'LLM output "{raw_text}" -> Switching to {detected_mode} mode')
                self.current_mode = detected_mode
                self.apply_parameters(detected_mode)
        elif "GO_LEFT" not in raw_text and "GO_RIGHT" not in raw_text:
             # Only warn if it wasn't a lane change command either
            self.get_logger().warn(f'Ignored invalid mode from LLM: "{raw_text}"')

    def vlm_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if not data.get("success", False):
                return
            
            decision = data.get("decision", "").lower()
            self.get_logger().info(f"Received VLM decision: {decision}")
            
            if decision == "center":
                # Car is in front, go left to overtake
                self.change_lane(1) 
            # If decision is "stay", "left", or "right", continue in current lane (do nothing)
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode VLM JSON: {msg.data}")

    def change_lane(self, lane_index):
        msg = Int32()
        msg.data = lane_index
        self.lane_publisher.publish(msg)
        lane_name = "LEFT" if lane_index == 1 else "RIGHT"
        self.get_logger().info(f'Lane Change Requested: {lane_name} ({lane_index})')

    def apply_parameters(self, mode):
        req = SetParameters.Request()
        
        # Define parameter sets for each mode
        if mode == "SAFETY":
            # Slow down, lower gains for stability
            req.parameters = [
                Parameter(name='v_scale', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.2)),
                Parameter(name='k_e', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)),
                 Parameter(name='k_h', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.5))
            ]
        elif mode == "AGGRESSIVE":
            # Speed up, higher gains for responsiveness
            req.parameters = [
                Parameter(name='v_scale', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)),
                Parameter(name='k_e', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)),
                 Parameter(name='k_h', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.5))
            ]
        else: # NORMAL
             req.parameters = [
                Parameter(name='v_scale', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.5)),
                Parameter(name='k_e', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)),
                 Parameter(name='k_h', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.5))
            ]

        # Call the service asynchronously
        if self.param_client.wait_for_service(timeout_sec=1.0):
            future = self.param_client.call_async(req)
            future.add_done_callback(self.param_response_callback)
        else:
            self.get_logger().warn('Controller parameter service not available!')

    def param_response_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Parameters updated successfully: {result}')
        except Exception as e:
            self.get_logger().error(f'Failed to update parameters: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
