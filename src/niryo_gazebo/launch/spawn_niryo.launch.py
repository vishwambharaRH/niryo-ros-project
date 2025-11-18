from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('niryo_gazebo')
    world_path = os.path.join(pkg_share, 'worlds', 'niryo_workspace.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ned.urdf.xacro')

    # Start Gazebo with ROS2 plugin
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_path}.items()
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'niryo'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=3.0, actions=[rsp_node]),
        TimerAction(period=5.0, actions=[spawn])
    ])