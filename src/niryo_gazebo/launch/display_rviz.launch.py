from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('niryo_gazebo')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ned.urdf.xacro')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # You can specify a config file here if you have one
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'niryo.rviz')]
    )

    return LaunchDescription([
        rsp_node,
        rviz_node
    ])