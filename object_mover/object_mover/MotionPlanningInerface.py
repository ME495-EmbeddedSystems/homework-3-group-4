import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose
from object_mover.RobotState import RobotState as CustomRobotState
from object_mover.MotionPlanner import MotionPlanner
from object_mover.PlanningScene import PlanningScene
from moveit_msgs.msg import RobotState as MoveitRobotState
from moveit_msgs.msg import MotionPlanRequest
from typing import Optional, List, Dict


class MotionPlanningInterface:
    '''
    A class that consolidates MotionPlanner, RobotState and PlanningScene classes in one class
    that is able to be utilized by any user node for any planning and execution task.
    '''
    def __init__(self, node: Node):
        '''
        Initializes MotionPlanningInterface with an instance of the ROS node passed to it as
        an argumet
        '''
        self.node = node
        self.planning_scene = PlanningScene(node = self.node)
        self.robot_state = CustomRobotState(node = self.node)
        self.motion_planner = MotionPlanner(node = self.node, robot_state = self.robot_state)
        self.path = MotionPlanRequest()
        
    async def plan_path(
        self,
        start_joints: Optional[List[float]] = None, 
        goal_joints: Optional [Dict[str, float]] = None,
        start_pose: Optional[Pose] = None, 
        goal_pose: Optional[Pose] = None,
        waypoints: List[Pose] = None,
        named_configuration: str = None,
        ):
        '''
        Function to plans path according to the inputs passed, uses MotionPlanner functions
        to plan path.
        :param start_joints: 
        '''
        if goal_joints is not None:
            self.path = await self.motion_planner.plan_joint_path(goal_joints)
        elif goal_pose is not None:
            self.path = await self.motion_planner.plan_pose_to_pose(
                start_pose,
                goal_pose
            )
        elif waypoints is not None:
            self.path = await self.motion_planner.plan_cartesian_path(waypoints)

        return self.path

    async def exec_path(self, path: MotionPlanRequest = None):
        if path is None:
            # Error message
            result = 0
        else:
            result = await self.motion_planner.execute_plan(path)
        return result
