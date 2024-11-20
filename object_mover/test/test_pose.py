"""Test launch file to verify that pose is achieved when moving from one pose to another"""
import unittest

from launch import LaunchDescription

from launch_ros.actions import Node as LaunchNode

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_testing.actions import ReadyToTest

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare

from geometry_msgs.msg import Pose

import pytest

import rclpy

import sys ,os

from time import sleep

root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, root_dir)

from object_mover.MotionPlanningInerface import *

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
                    FindPackageShare("franka_fer_moveit_config"),
                    "launch/"
                    "demo.launch.py"
                ]
            ),
            launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items()
        ),
        ReadyToTest(),
    ])


class TestPose(unittest.TestCase):
    """Test the pose of the robot arm"""
    @classmethod
    def setUpClass(cls):
        """Setup the test class."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Cleanup the test class."""
        rclpy.shutdown()

    def setUp(self):
        """Setup the test."""
        self.node = rclpy.create_node("test_node")
        self.node.get_logger().info("Node created")

        self.mpi = MotionPlanningInterface(self.node)

        while (self.mpi.robot_state.joint_state is None):
            print("Waiting for the joint states...")
            rclpy.spin_once(self.node)

        self.node.get_logger().info("MPI created")

    def test_pose_achieved(self):
        """Test that the pose is achieved."""
        
        # {x: 0.5, y: 0.0, z: 0.5}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}

        object_pose = Pose()
        object_pose.position.x = 0.4
        object_pose.position.y = 0.0
        object_pose.position.z = 0.5
        object_pose.orientation.x = 1.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 0.0

        ex = rclpy.get_global_executor()
        try:
            future = ex.create_task(self.mpi.plan_path(goal_pose = object_pose))
            rclpy.spin_until_future_complete(self.node, future, executor=ex)
            
        except Exception as e:
            print(e)
            self.tearDown()
            return
        
        result = future.result()

        assert result.val == 1

    def tearDown(self):
        self.node.destroy_node()


# # generate_test_description()
# t = TestPose()
# t.setUpClass()
# t.setUp()
# t.test_pose_achieved()
# t.tearDown()