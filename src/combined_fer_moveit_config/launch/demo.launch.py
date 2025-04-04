from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fer", package_name="combined_fer_moveit_config").to_moveit_configs()
    description = generate_demo_launch(moveit_config)

    # We need a joint_state_publisher to unify the joint states from the hand and the arm
    description.add_action(
        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             parameters=[{'source_list': ['joint_state_broadcaster/joint_states', 'hand/joint_states'], 'rate': 30}]))
    return description
