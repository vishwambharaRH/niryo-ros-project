from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch gesture control interface"""
    
    pkg_dir = get_package_share_directory('gesture_control_interface')
    
    # Launch arguments
    control_mode = DeclareLaunchArgument(
        'control_mode',
        default_value='basic',
        description='Gesture control mode (basic, advanced, custom)'
    )
    
    enable_command_publisher = DeclareLaunchArgument(
        'enable_command_publisher',
        default_value='true',
        description='Enable command publisher utility node'
    )
    
    # Gesture to Command Node
    gesture_to_command_node = Node(
        package='gesture_control_interface',
        executable='gesture_to_command',
        name='gesture_to_command',
        parameters=[
            {'control_mode': LaunchConfiguration('control_mode')},
            {'gesture_mappings_file': os.path.join(pkg_dir, 'config', 'gesture_mappings.yaml')}
        ],
        output='screen'
    )
    
    # Command Publisher Node (optional)
    command_publisher_node = Node(
        package='gesture_control_interface',
        executable='command_publisher',
        name='command_publisher',
        parameters=[
            {'enable_logging': True},
            {'output_format': 'json'}
        ],
        output='screen',
        condition=lambda context: context.launch_configurations['enable_command_publisher'] == 'true'
    )
    
    return LaunchDescription([
        control_mode,
        enable_command_publisher,
        gesture_to_command_node,
        command_publisher_node
    ])