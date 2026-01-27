from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Allow users to change sensitivity from the terminal
    threshold_arg = DeclareLaunchArgument(
        'hysteresis_threshold',
        default_value='5',
        description='Frames needed to confirm VLM input'
    )

    decision_node = Node(
        package='overtake_decision',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[{
            'hysteresis_threshold': LaunchConfiguration('hysteresis_threshold'),
        }]
    )

    return LaunchDescription([
        threshold_arg,
        decision_node
    ])