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
from typing import List

from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import RobotState as MoveitRobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK

from object_mover.utils import home_joint_positions, robot_joints

from rclpy.node import Node

from sensor_msgs.msg import JointState


class RobotState:
    """Class that can compute forward or inverse kinematics of the robot."""

    def __init__(self, node: Node):
        self.node = node
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.fk_client = self.node.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        self.joint_state = None
        self.home_joint_state = dict(zip(robot_joints(), home_joint_positions()))

    def joint_state_callback(self, msg):
        """
        Joint state publisher callback.

        :param msg: Joint states of the robotic system
        :type msg: sensor_msgs.msg.JointState
        """
        self.joint_state = msg

    async def compute_IK(
        self,
        goal_pose: PoseStamped,
        joint_state: JointState = None,
        group_name: str = 'fer_manipulator',
    ):
        """
        Compute inverse kinematics of the system.

        :param goal_pose: goal position of end-effector
        :type goal_pose: geometry_msgs.msg.PoseStamped
        :param joint_state: joint state configuration of robot
        :type joint_state: sensor_msgs.msg.JointState
        :param group_name: name of group of links and joints
        :type group_name: string
        :returns: Inverse Kinematics solution
        :rtype: moveit_msgs/RobotState
        """
        if joint_state is None:
            joint_state = self.joint_state
        request = GetPositionIK.Request()
        request.ik_request.ik_link_name = 'fer_hand_tcp'
        request.ik_request.group_name = group_name
        request.ik_request.robot_state.joint_state = joint_state
        request.ik_request.pose_stamped.pose = goal_pose
        ik_solution = await self.ik_client.call_async(request)
        return ik_solution

    async def compute_FK(self, link_names: List[str], joint_state: JointState = None):
        """
        Compute forward kinematics of the system or from current configuration.

        :param link_names: link  names to compute FK for
        :type link_names: list[string]
        :param joint_state: joint states of robot
        :type joint_state: sensor_msgs.msg.JointState
        :returns: Forward Kinematics solution
        :rtype: moveit_msgs/RobotState
        """
        if joint_state is None:
            joint_state = self.joint_state
        request = GetPositionFK.Request()
        request.fk_link_names = link_names
        request.robot_state.joint_state = joint_state
        fk_solution = await self.fk_client.call_async(request)
        return fk_solution

    def get_robot_state(self) -> MoveitRobotState:
        """
        Acquires current robot state.

        :returns: Robot state (moveit's msg definition)
        :rtype: moveit_msgs/RobotState
        """
        robot_state = MoveitRobotState()
        robot_state.joint_state = self.joint_state
        return robot_state
