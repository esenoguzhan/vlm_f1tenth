from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    stanley_dir = get_package_share_directory('stanley_control')
    
    # Declare parameters so they can be overridden from CLI
    # The default paths uses the installed racelines via ament_index (handled in cpp, but we can also set here)
    # Since the C++ node now uses ament_index to find defaults if not provided, 
    # we don't strictly need to pass paths unless we want to override them.
    # But let's expose these for flexibility.

    return LaunchDescription([
        Node(
            package='stanley_control',
            executable='stanley_node',
            name='stanley_controller',
            output='screen',
            parameters=[{
                'k_e': 1.5,
                'k_h': 1.0,
                'v_scale': 0.8,
                'wheelbase': 0.33,
                # CSV paths are handled automatically by the C++ node defaults using ament_index
                # unless overridden here.
                'right_csv': os.path.join(stanley_dir, 'racelines', 'min_curve.csv'),
            }]
        )
    ])
