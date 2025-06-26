from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('hand_rviz')
    urdf_path = os.path.join(pkg_share, 'urdf', 'hand_urdf.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # Launch configuration for js_gui flag
    js_gui = LaunchConfiguration('js_gui')
    rviz_config = LaunchConfiguration('rviz_config')
    calibrating_offset = LaunchConfiguration('calibrating_offset')
    is_calibrated = LaunchConfiguration('is_calibrated')

    return LaunchDescription([
        DeclareLaunchArgument(
            'js_gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui (true) or joint_state_publisher (false)'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'config', 'urdf.rviz'),
            description='Path to the RViz config file'
        ),
        DeclareLaunchArgument(
            'calibrating_offset',
            default_value='false',
            description='When true, disables joint_state_forwarding node'
        ),
        DeclareLaunchArgument(
            'is_calibrated',
            default_value='true',
            description='Whether to use calibrated servo input'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        # Launch joint_state_publisher_gui if js_gui is true
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(js_gui)
        ),
        # Launch joint state forwarding node if js_gui is false AND calibrating_offset is false
        Node(
            package='hand_rviz',
            executable='joint_state_forwarding',
            output='screen',
            # Only run when js_gui is false
            condition=UnlessCondition(js_gui),
            parameters=[
                {'is_calibrated': is_calibrated},
                {'calibrating_offset': calibrating_offset}
            ]
        ),
        # We need a joint_state_publisher 
        Node(package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'source_list': ['hand/joint_states'], 'rate': 30}],
            condition=UnlessCondition(js_gui)),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
