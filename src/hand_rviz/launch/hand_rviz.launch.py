from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('hand_rviz')
    urdf_path = os.path.join(pkg_share, 'urdf', 'hand_urdf.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'urdf.rviz')]
        ),
    ])
