from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch gesture detection system with camera"""
    
    pkg_dir = get_package_share_directory('gesture_recognition')
    
    # Launch arguments
    use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Use camera handler (false if camera already running)'
    )
    
    camera_index = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index'
    )
    
    debug_image = DeclareLaunchArgument(
        'debug_image',
        default_value='true',
        description='Publish debug visualization'
    )
    
    use_classifier = DeclareLaunchArgument(
        'use_classifier',
        default_value='true',
        description='Use gesture classifier for temporal filtering'
    )
    
    # Nodes
    camera_node = Node(
        package='gesture_recognition',
        executable='camera_handler',
        name='camera_handler',
        parameters=[
            os.path.join(pkg_dir, 'config', 'camera_params.yaml'),
            {'camera_index': LaunchConfiguration('camera_index')}
        ],
        condition=lambda context: context.launch_configurations['use_camera'] == 'true'
    )
    
    detector_node = Node(
        package='gesture_recognition',
        executable='gesture_detector',
        name='gesture_detector',
        parameters=[
            os.path.join(pkg_dir, 'config', 'gesture_params.yaml'),
            {'publish_debug_image': LaunchConfiguration('debug_image')}
        ],
        output='screen'
    )
    
    classifier_node = Node(
        package='gesture_recognition',
        executable='gesture_classifier',
        name='gesture_classifier',
        parameters=[
            os.path.join(pkg_dir, 'config', 'gesture_params.yaml')
        ],
        output='screen',
        condition=lambda context: context.launch_configurations['use_classifier'] == 'true'
    )
    
    return LaunchDescription([
        use_camera,
        camera_index,
        debug_image,
        use_classifier,
        camera_node,
        detector_node,
        classifier_node
    ])
