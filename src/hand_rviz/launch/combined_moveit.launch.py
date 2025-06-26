#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    Shutdown
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
import xacro

import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    pkg_share = get_package_share_directory('hand_rviz')
    xacro_file = os.path.join(pkg_share, 'urdf', 'combined_description.xacro')

    # Process the xacro file to URDF
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # Load the plannning pipeline
    ompl_planning_pipeline_config = {
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames',
                                'default_planning_request_adapters/ValidateWorkspaceBounds',
                                'default_planning_request_adapters/CheckStartStateBounds',
                                'default_planning_request_adapters/CheckStartStateCollision'],
            'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization',
                                  'default_planning_response_adapters/ValidateSolution',
                                  'default_planning_response_adapters/DisplayMotionPath'],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['ompl'].update(load_yaml(
        'franka_fer_moveit_config', 'config/ompl_planning.yaml'))
    # robot_description = ParameterValue(Command([ExecutableInPackage("xacro", "xacro"), " ",
                                                # PathJoinSubstitution([FindPackageShare("franka_description"), "robots", "fer", "fer.urdf.xacro"]),
                                                # ' hand:=true',
                                                # ' robot_ip:=', LaunchConfiguration('robot_ip'),
                                                # ' use_fake_hardware:=',LaunchConfiguration('use_fake_hardware'),
                                                # ' fake_sensor_commands:=',LaunchConfiguration('fake_sensor_commands'),
                                                # ' ros2_control:=true']),value_type=str)

    robot_description_semantic = ParameterValue(Command([ExecutableInPackage("xacro", "xacro"), ' ',
                                                        PathJoinSubstitution([FindPackageShare('hand_rviz'),'srdf','combined.srdf.xacro'])])
                                               , value_type=str)
    kinematics_yaml = {"robot_description_kinematics" : load_yaml('franka_fer_moveit_config', 'config/kinematics.yaml')}

    return LaunchDescription(
        [ DeclareLaunchArgument('robot_ip', default_value='', description='Hostname or IP address of the robot.')
         ,DeclareLaunchArgument('use_fake_hardware', default_value='false',description='Use fake hardware')
         ,DeclareLaunchArgument('fake_sensor_commands', default_value='false', description='Fake sensor commands. Only valid when "use_fake_hardware" == true')
         ,DeclareLaunchArgument('db', default_value='False', description='Database flag')

          ,Node(package='rviz2',
                executable='rviz2',
                output='log',
                arguments=['-d', PathJoinSubstitution([FindPackageShare('franka_fer_moveit_config'),'rviz','moveit.rviz'])],
                parameters=[kinematics_yaml,
                            {'robot_description_planning': load_yaml('franka_fer_moveit_config', 'config/joint_limits.yaml')}
                            ]
              )
         ,Node(package='robot_state_publisher',
              executable='robot_state_publisher',
              name='robot_state_publisher',
              output='both',
              parameters=[{'robot_description': robot_description_content}]
            )
          ,SetLaunchConfiguration('fer_controllers',
                                  value=PathJoinSubstitution([FindPackageShare('franka_fer_moveit_config'), 'config', 'fer_real_controllers.yaml']),
                                  condition=IfCondition(LaunchConfiguration('use_fake_hardware')))
          ,SetLaunchConfiguration('fer_controllers',
                                  value=PathJoinSubstitution([FindPackageShare('franka_fer_moveit_config'), 'config', 'fer_real_controllers.yaml']),
                                  condition=UnlessCondition(LaunchConfiguration('use_fake_hardware'))
                                  )
          ,Node(package='controller_manager',
                executable='ros2_control_node',
                parameters=[
                    {'arm_id': 'fer',
                     'robot_description': robot_description_content
                    },
                    LaunchConfiguration('fer_controllers')],
                remappings=[('joint_states', 'franka/joint_states')],
                output='screen',
                on_exit=Shutdown(),
                )
          ,Node(package='joint_state_publisher',
                executable='joint_state_publisher',
                parameters=[{'source_list': ['franka/joint_states', 'fer_gripper/joint_states'], 'rate': 30}],
          )
          ,Node(package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    {'publish_robot_description_semantic': True,
                     'allow_trajectory_execution' : True,
                     'publish_planning_scene': True,
                     'publish_state_updates': True,
                     'publish_transforms_updates': True,
                     'publish_geometry_updates': True,
                     'monitor_dynamics': False,
                     'robot_description': robot_description_content,
                     'robot_description_semantic': robot_description_semantic,
                     'robot_description_planning': load_yaml('franka_fer_moveit_config', 'config/joint_limits.yaml')
                     },
                    kinematics_yaml,
                    # Setup the planning pipeline
                    ompl_planning_pipeline_config,
                    {'default_planning_pipeline': 'ompl',
                     'planning_pipelines': ['ompl']
                    },
                    # Allow moveit to manage trajectory execution
                    {'moveit_simple_controller_manager': load_yaml('franka_fer_moveit_config', 'config/fer_real_controllers.yaml'),
                     'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                     'moveit_manage_controllers': True,
                     'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                     'trajectory_execution.allowed_goal_duration_margin': 0.5,
                     'trajectory_execution.allowed_start_tolerance': 0.01,
                     }
                ]
          )
          ,Node(package='controller_manager',
               executable='spawner',
               arguments=['franka_robot_state_broadcaster'],
               output='screen',
               condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')))
          ,ExecuteProcess(cmd=[ExecutableInPackage('spawner', 'controller_manager'),' fer_arm_controller'],
                          shell=True, output='screen')
          ,ExecuteProcess(cmd=[ExecutableInPackage('spawner', 'controller_manager'),' joint_state_broadcaster'],
                          shell=True, output='screen')
         ]
    )
