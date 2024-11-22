# Copyright 2024 David davidkh@u.northwestern.edu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        'fer', package_name='franka_fer_moveit_config'
    ).to_moveit_configs()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'debug',
                default_value='false',
                description='Boolean flag to indicate production mode: true or false.',
            ),
            Node(
                package='rviz2',
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('debug'), 'true')
                ),
                executable='rviz2',
                output='log',
                arguments=[
                    '-d',
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_fer_moveit_config'),
                            'config',
                            'moveit.rviz',
                        ]
                    ),
                ],
                parameters=[
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                ],
                remappings=[
                    (
                        '/move_action/_action/feedback',
                        '/viz/move_action/_action/feedback',
                    ),
                    ('/move_action/_action/status', '/viz/move_action/_action/status'),
                    (
                        '/move_action/_action/cancel_goal',
                        '/viz/move_action/_action/cancel_goal',
                    ),
                    (
                        '/move_action/_action/get_result',
                        '/viz/move_action/_action/get_result',
                    ),
                    (
                        '/move_action/_action/send_goal',
                        '/viz/move_action/_action/send_goal',
                    ),
                ],
            ),
            Node(
                package='object_mover',
                executable='pick_node',
            ),
        ]
    )
