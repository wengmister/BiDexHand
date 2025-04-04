import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('hand_rviz')
    xacro_file = os.path.join(pkg_share, 'urdf', 'combined_description.xacro')

    # Process the xacro file to URDF
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # Launch configuration for js_gui flag
    js_gui = LaunchConfiguration('js_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'js_gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui (true) or joint_state_publisher (false)'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),
        # Launch joint_state_publisher_gui if js_gui is true
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(js_gui)
        ),
        # Launch joint state fusion node if js_gui is false
        Node(
            package='hand_rviz',
            executable='joint_state_fusion',
            output='screen',
            condition=UnlessCondition(js_gui)
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'combined.rviz')],
            output='screen'
        ),
    ])
