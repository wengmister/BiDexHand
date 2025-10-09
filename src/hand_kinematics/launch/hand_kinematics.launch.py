from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    hand_rviz_dir = get_package_share_directory('hand_rviz')
    
    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(hand_rviz_dir, 'urdf', 'hand_urdf.urdf'),
        description='Path to URDF file'
    )
    
    # Create the hand kinematics node
    hand_kinematics_node = Node(
        package='hand_kinematics',
        executable='hand_kinematics_node',
        name='hand_kinematics',
        output='screen',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_file')
        }]
    )
    
    return LaunchDescription([
        urdf_file_arg,
        hand_kinematics_node
    ])