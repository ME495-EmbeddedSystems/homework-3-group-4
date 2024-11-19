"""
Node that conducts pick and drop task.

The pick node is a node that accomplishes the pick and place task of an object
by the robotic arm. It directly interacts with MotionPlanningInterface to
access the functions of the objects of MotionPlanner, RobotState and
PlanningScene incorporated within the MotionPlanningInterface class.
"""

import copy
from enum import Enum, auto

from geometry_msgs.msg import Pose

from moveit_msgs.action import MoveGroup

from object_mover.MotionPlanningInerface import MotionPlanningInterface

from object_mover_interfaces.srv import PickPose

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node


class State(Enum):
    """Current state of the system (might not be needed)."""

    MOVING = auto()
    STATIONARY = auto()


class PickNode(Node):
    """
    Node that conducts pick and drop task.

    Node that interacts with the moveit API to conduct pick and place task
    with robot arm.
    """

    def __init__(self):
        """Define parameters."""
        super().__init__('pick_node')
        self._cbgroup = MutuallyExclusiveCallbackGroup()
        self.serv = self.create_service(
            PickPose,
            'pick',
            self.pick_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        self._server = ActionServer(self,
                                    MoveGroup,
                                    '/viz/move_action',
                                    self.move_action_callback,
                                    callback_group=self._cbgroup)
        self.state = State.STATIONARY
        self.mpi = MotionPlanningInterface(self)

    async def pick_callback(self, request: PickPose, response):
        """
        Execute pick taskflow.

        :param request: service request object
        :type request: object_mover_interfaces.srv.PickPose
        :returns: response to service call
        :rtype: bool
        """
        box_pose = Pose()
        box_pose.position.x = 0.59
        box_pose.position.y = 0.13
        box_pose.position.z = 0.1378
        await self.mpi.planning_scene.add_collision_objects(
            'box',
            box_pose,
            [0.05, 0.05, 0.05])

        wall_pose = Pose()
        wall_pose.position.x = 0.5
        wall_pose.position.y = 0.3
        wall_pose.position.z = 0.0

        await self.mpi.planning_scene.add_collision_objects(
            'wall',
            wall_pose,
            [0.05, 0.05, 1.0])
        object_pose = Pose()
        object_pose = request.pick_point
        pose1 = copy.deepcopy(object_pose)

        pose1.position.z = object_pose.position.z + 0.4
        await self.mpi.plan_path(goal_pose=pose1)

        await self.mpi.motion_planner.toggle_gripper('open')

        pose2 = object_pose
        await self.mpi.plan_path(goal_pose=pose2)

        # Closing grippers
        await self.mpi.motion_planner.toggle_gripper('close')

        # Attaching box to arm in scene
        await self.mpi.planning_scene.attach_object('box')

        # Lifts object slighty off table
        pose3 = object_pose
        pose3.position.z = object_pose.position.z + 0.3
        await self.mpi.plan_path(goal_pose=pose3)

        # Move arm to other side of obstacle
        pose4 = object_pose
        pose4.position.y = object_pose.position.y + 0.3
        await self.mpi.plan_path(goal_pose=pose4)

        # Drop object
        await self.mpi.motion_planner.toggle_gripper('open')
        return response

    async def move_action_callback(self, goal_handle):
        """
        Move action callback.

        :param goal_handle: goal handle of move action
        :type goal_handle:
        :rtype: bool
        """
        self.get_logger().info('Move Action Called. Goal Is')
        self.get_logger().info('--- End ofrequest dump.----\n')
        self.get_logger().info(
            'Forwarding the action to the move_group action server')

        self.get_logger().info('Awaiting the result')
        self.get_logger().info('Interception successful, returning the result')
        return


def pick_entry(args=None):
    """Execute main function."""
    rclpy.init(args=args)
    node = PickNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    pick_entry()
