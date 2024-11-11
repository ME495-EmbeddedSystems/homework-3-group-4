from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, Constraints, MotionPlanRequest
from typing import Optional, List


class MotionPlanner:
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

    async def plan_joint_path(self, start_joints: Optional[List[float]], goal_joints: List[float]):
        """
        Plan a path from a starting joint configuration to a goal joint configuration.

        :param start_joints: Starting joint angles. Uses current robot state if not provided.
        :type start_joints: List[float], optional
        :param goal_joints: Goal joint angles.
        :type goal_joints: List[float]
        :returns: The planned motion path request.
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        pass

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
        pass

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
