"""Launchfile for the real robot. Does not start Rviz by default."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch, generate_move_group_launch, generate_moveit_rviz_launch, generate_spawn_controllers_launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("fer", package_name="combined_fer_moveit_config").
        robot_description(mappings={"hand" : "false",
                                    "use_fake_hardware" : "false",
                                    "fake_sensor_commands" : "false",
                                    "ros2_control" : "true",
                                    "robot_ip" : LaunchConfiguration("robot_ip")}).to_moveit_configs()
    )
    return LaunchDescription(generate_rsp_launch(moveit_config).entities
    + generate_move_group_launch(moveit_config).entities
    + generate_spawn_controllers_launch(moveit_config).entities
    + [DeclareLaunchArgument("robot_ip", description="URL or ip address for the robot."),
       DeclareLaunchArgument("use_rviz", default_value="False", description="To use rviz, set to true."),
       IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('combined_fer_moveit_config'), 'launch', 'moveit_rviz.launch.py']),
                                condition=IfCondition(LaunchConfiguration('use_rviz'))),
    Node(package="controller_manager",
         executable="ros2_control_node",
         parameters=[PathJoinSubstitution([FindPackageShare('combined_fer_moveit_config'),'config','fer_real_controllers.yaml'])],
         remappings=[("/controller_manager/robot_description", "/robot_description")],),
    Node(package='joint_state_publisher',
         executable='joint_state_publisher',
         parameters=[{'source_list': ['joint_state_broadcaster/joint_states', 'hand/joint_states'], 'rate': 1000}])
    ])

