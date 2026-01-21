import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        # Subscribe to LLM output
        self.subscription = self.create_subscription(
            String,
            'output',
            self.mode_callback,
            10)
        
        # Client to set parameters on the controller node
        self.param_client = self.create_client(SetParameters, '/stanley_controller/set_parameters')
        
        self.current_mode = "NORMAL"
        self.get_logger().info('Decision Node started. Waiting for driving mode from LLM...')

    def mode_callback(self, msg):
        raw_text = msg.data.upper()
        detected_mode = None

        # Prioritize SAFETY check
        if "SAFETY" in raw_text:
            detected_mode = "SAFETY"
        elif "AGGRESSIVE" in raw_text:
            detected_mode = "AGGRESSIVE"
        elif "NORMAL" in raw_text:
            detected_mode = "NORMAL"

        if detected_mode:
            if detected_mode != self.current_mode:
                self.get_logger().info(f'LLM output "{msg.data}" -> Switching to {detected_mode} mode')
                self.current_mode = detected_mode
                self.apply_parameters(detected_mode)
        else:
            self.get_logger().warn(f'Ignored invalid mode from LLM: "{msg.data}"')

    def apply_parameters(self, mode):
        req = SetParameters.Request()
        
        # Define parameter sets for each mode
        if mode == "SAFETY":
            # Slow down, lower gains for stability
            req.parameters = [
                Parameter(name='v_scale', value=ParameterType(type=ParameterType.PARAMETER_DOUBLE, double_value=0.5)),
                Parameter(name='k_e', value=ParameterType(type=ParameterType.PARAMETER_DOUBLE, double_value=0.5)),
                 Parameter(name='k_h', value=ParameterType(type=ParameterType.PARAMETER_DOUBLE, double_value=0.8))
            ]
        elif mode == "AGGRESSIVE":
            # Speed up, higher gains for responsiveness
            req.parameters = [
                Parameter(name='v_scale', value=ParameterType(type=ParameterType.PARAMETER_DOUBLE, double_value=1.5)),
                Parameter(name='k_e', value=ParameterType(type=ParameterType.PARAMETER_DOUBLE, double_value=2.0)),
                 Parameter(name='k_h', value=ParameterType(type=ParameterType.PARAMETER_DOUBLE, double_value=1.2))
            ]
        else: # NORMAL
             req.parameters = [
                Parameter(name='v_scale', value=ParameterType(type=ParameterType.PARAMETER_DOUBLE, double_value=0.8)),
                Parameter(name='k_e', value=ParameterType(type=ParameterType.PARAMETER_DOUBLE, double_value=1.5)),
                 Parameter(name='k_h', value=ParameterType(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0))
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
