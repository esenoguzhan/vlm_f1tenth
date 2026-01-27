import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Locate the config file path
    pkg_share = get_package_share_directory('vlm_package')
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')

    # 2. Define Launch Arguments
    launch_args = [
        DeclareLaunchArgument(
            'api',
            default_value='mlc',
            description='The model backend to use (mlc, auto, tensorrt)'),
        DeclareLaunchArgument(
            'quantization',
            default_value='q4f16_ft',
            description='The quantization method to use'),
    ]

    api_config = LaunchConfiguration('api')
    quant_config = LaunchConfiguration('quantization')

    # 3. Model Node (VLM Node)
    vlm_node = Node(
        package='vlm_package',
        executable='vlm_node',
        name='vlm_node',  # CHANGED: Must match the top-level key in config.yaml
        output='screen',
        parameters=[
            config_file, 
            {
                'api': api_config,
                'quantization': quant_config,
            }
        ],
        remappings=[
            # Map internal names to actual vehicle topics
            ('input_image', '/zed/zed_node/left/image_rect_color'), 
            ('slam_pose', '/pose') # Standard output for slam_toolbox
        ]
    )

    # Note: We are NOT launching the ZED node here. 
    # It is better to launch the ZED and SLAM separately to ensure 
    # they are stable before the VLM starts its heavy loading.

    return LaunchDescription(launch_args + [vlm_node])