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
"""Test launch file to verify that pose is achieved when moving from one pose to another."""
from time import sleep
import unittest

from geometry_msgs.msg import Pose

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest

from object_mover.MotionPlanningInterface import MotionPlanningInterface
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    """Launch the nodes under test."""
    # Here we launch the frand the move group
    return LaunchDescription([
        # Launch the launch file demo.launch.py
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Set to true to launch RViz'
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    FindPackageShare('franka_fer_moveit_config'),
                    'launch/'
                    'demo.launch.py'
                ]
            ),
            launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items()
        ),
        ReadyToTest(),
    ])


class TestPose(unittest.TestCase):
    """Test the pose of the robot arm."""

    @classmethod
    def setUpClass(cls):
        """Initialize the test class."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Cleanup the test class."""
        rclpy.shutdown()

    def setUp(self):
        """Initialize the test."""
        # Wait for Moveit to launch
        sleep(10)
        self.node = rclpy.create_node('test_node')

        self.mpi = MotionPlanningInterface(self.node)

        while (self.mpi.robot_state.joint_state is None):
            # print("Waiting for the joint states...")
            rclpy.spin_once(self.node)

    def test_pose_achieved(self):
        """Test that the pose is achieved."""
        object_pose = Pose()
        object_pose.position.x = 0.4
        object_pose.position.y = 0.0
        object_pose.position.z = 0.5
        object_pose.orientation.x = 1.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 0.0
        # .......... Begin_Citation [1] .......... #
        ex = rclpy.get_global_executor()

        future = ex.create_task(self.mpi.plan_path(goal_pose=object_pose))
        rclpy.spin_until_future_complete(self.node, future, executor=ex)
        # .......... End_Citation [1] .......... #
        result = future.result()

        # SUCCESS=1
        assert result.val == 1

    def tearDown(self):
        self.node.destroy_node()
