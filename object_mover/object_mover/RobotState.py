"""
Computes forward and inverse kinematics.

This class is used by MotionPlanner to find current FK
and IK solutions to aid in path planning. The
MotionPlanningInterface class incorporates RobotState
as part of its overall functionality.
"""


from typing import List

from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import RobotState as MoveitRobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK

from rclpy.node import Node

from sensor_msgs.msg import JointState


class RobotState:
    """
    Class that computes FK and IK solutions within it.

    A node is passed to RobotState that subscribers to joint states and is
    used to compute IK and FK solutions.
    """

    def __init__(self, node: Node):
        """Declare subscriber for joint states and clients for FK and IK."""
        self.node = node
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback, 10)
        self.fk_client = self.node.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        self.joint_state = None

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
            group_name: str = 'fer_manipulator'):
        """
        Compute IK from current joint states or given goal pose.

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

    async def compute_FK(
            self,
            link_names: List[str],
            joint_state: JointState = None):
        """
        Compute FK from given current/given joint states.

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
        Acquire current robot state.

        :returns: Robot state (moveit's msg definition)
        :rtype: moveit_msgs/RobotState
        """
        robot_state = MoveitRobotState()
        robot_state.joint_state = self.joint_state
        return robot_state
