from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, Constraints, MotionPlanRequest, JointConstraint, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from typing import Optional, List, Dict
import utils



class MotionPlanner(Node):
    """
    A class for planning motion paths using MoveIt.

    This class provides methods for planning different types of motion paths:
    - Joint configuration to joint configuration
    - Pose to pose
    - Cartesian paths
    - Named configuration paths

    Attributes
    ----------
    client : ActionClient
        The action client used to communicate with the MoveIt action server.
    """

    def __init__(self):
        """Initialize the MotionPlanner class."""
        self.client = ActionClient(self, MoveGroup, 'move_action')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')

        if not self.client.wait_for_server(timeout_sec=10):
            raise RuntimeError('MoveGroup action server not ready')

    async def execute_plan(self, plan):
        """
        Execute a previously planned motion.

        :param plan: The planned motion to be executed.
        :type plan: moveit_msgs.msg.MotionPlanRequest
        :returns: True if execution is successful, False otherwise.
        :rtype: bool
        """
        goal_handle = await self.client.send_goal_async(plan)
        if not goal_handle.accepted:
            return False

        result = await goal_handle.get_result_async()
        return result.result.error_code == 0

    async def plan_joint_path(self, start_joints: Optional[List[float]], goal_joints: Dict[str, float]): # noqa 501
        """
        Plan a path from a valid starting joint configuration to a valid goal joint configuration.

        :param start_joints: Starting joint angles. Uses current robot state if not provided.
        :type start_joints: List[float], optional
        :param goal_joints: Goal joint angles.
        :type goal_joints: Dict[str, float]
        :returns: The planned motion path request.
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        path = MotionPlanRequest()
        path.start_state.joint_state = JointState()

        if start_joints:
            path.start_state.joint_state.position = start_joints

        else:
            # use the current robot state if the starting joint angles are not provided
            current_state = await self.get_current_robot_state()
            path.start_state.joint_state.position = current_state.joint_state.position

        path.goal_constraints = [Constraints()]
        path.goal_constraints[0].joint_constraints = [
            JointConstraint(
                joint_name=joint,
                position=angle,
                tolerance_below=0.0001,
                tolerance_above=0.0001,
                weight=1.0
            )
            for joint, angle in goal_joints.items()
        ]
        return path

    async def plan_pose_to_pose(self, start_pose: Optional[Pose], goal_pose: Optional[Pose]):
        """
        Plan a path from a starting pose to a goal pose.

        :param start_pose: Starting pose of the robot. Uses current robot state if not provided.
        :type start_pose: geometry_msgs.msg.Pose, optional
        :param goal_pose: Goal pose of the robot.
        :type goal_pose: geometry_msgs.msg.Pose, optional
        :returns: The planned motion path request.
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        path = MotionPlanRequest()

        if not start_pose:
            current_state = await self.get_current_robot_state()
            path.start_state.joint_state.position = current_state.joint_state.position

        if not goal_pose.position: # finish the points 2.2 and 2.3
            ikrequest = PositionIKRequest()
            ikrequest.group_name = 'fer_joint7'
            ikrequest.pose_stamped.pose = goal_pose
            current_state = await self.get_current_robot_state()
            ikrequest.robot_state = current_state

            computed_joint_constraints = Constraints()
            joint_constraint = JointConstraint()

            path_solution = await self.ik_client.call_async(ikrequest)
            for index, name in path_solution.joint_state.name:
                position = path_solution.joint_state.positon[index]
                joint_constraint.joint_name = name
                joint_constraint.position = position
                joint_constraint.tolerance_above = 0.0001
                joint_constraint.tolerance_below = 0.0001
                joint_constraint.weight = 1.0
                computed_joint_constraints.joint_constraints.append(joint_constraint)

        path.goal_constraints = computed_joint_constraints
        path.start_state = current_state

        return path
        

    async def plan_cartesian_path(self, waypoints: List[Pose]):
        """
        Plan a Cartesian path from a sequence of waypoints.

        :param waypoints: A list of poses that define the Cartesian path.
        :type waypoints: List[geometry_msgs.msg.Pose]
        :returns: The planned motion path request.
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        pass

    async def plan_to_named_configuration(self, named_configuration: str):
        """
        Plan a path to a named configuration.

        :param named_configuration: The name of the configuration to plan to.
        :type named_configuration: str
        :returns: The planned motion path request.
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        pass

    def save_plan(self, plan):
        """
        Save a motion plan for future execution.

        :param plan: The motion plan to save.
        :type plan: moveit_msgs.msg.MotionPlanRequest
        """
        pass

    def inspect_plan(self, plan):
        """
        Inspect a previously saved motion plan.

        :param plan: The motion plan to inspect.
        :type plan: moveit_msgs.msg.MotionPlanRequest
        """
        pass

    async def get_current_robot_state(self) -> RobotState:
        """
        Get the current state of the robot.

        :returns: The current state of the robot.
        :rtype: moveit_msgs.msg.RobotState
        """
        pass
