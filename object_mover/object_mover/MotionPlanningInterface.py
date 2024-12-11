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
from typing import Dict, List, Optional

from geometry_msgs.msg import Pose

from moveit_msgs.msg import MotionPlanRequest

from object_mover.MotionPlanner import MotionPlanner
from object_mover.PlanningScene import PlanningScene
from object_mover.RobotState import RobotState as CustomRobotState

from rclpy.node import Node


class MotionPlanningInterface:
    """A class that consolidates MotionPlanner, RobotState and PlanningScene classes."""

    def __init__(self, node: Node):
        """Initialize MotionPlanningInterface with an instance of the ROS node."""
        self.node = node
        self.planning_scene = PlanningScene(node=self.node)
        self.robot_state = CustomRobotState(node=self.node)
        self.motion_planner = MotionPlanner(
            node=self.node,
            robot_state=self.robot_state,
            planning_scene=self.planning_scene,
        )

    async def plan_path(
        self,
        start_joints: Optional[List[float]] = None,
        goal_joints: Optional[Dict[str, float]] = None,
        start_pose: Optional[Pose] = None,
        goal_pose: Optional[Pose] = None,
        waypoints: List[Pose] = None,
        named_configuration: str = None,
    ):
        """
        Plan path according to the inputs passed, uses MotionPlanner functions.

        :param start_joints: starting joint configuration for path planning
        :type start_joints: list[float]
        :param goal_joints: goal joint configuration for path planning
        :type goal_joints: dict[string, float]
        :param start_pose: starting end-effector position for path planning
        :type start_pose: geometry_msgs.msg.Pose
        :param goal_pose: goal end-effector position for path planning
        :type goal_pose: geometry_msgs.msg.Pose
        :param waypoints: list of waypoints for cartesian path planning
        :type waypoints: list[geometry_msgs.msg.Pose]
        :param named_configuration: named configuration input passed for path planning
        :type named_configuration: string
        :returns: planned path
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        if goal_joints is not None:
            path = await self.motion_planner.plan_joint_path(
                start_joints=start_joints, goal_joints=goal_joints, execute=True
            )
        elif goal_pose is not None:
            path = await self.motion_planner.plan_pose_to_pose(
                start_pose, goal_pose, execute=False
            )
        elif waypoints is not None:
            path = await self.motion_planner.plan_cartesian_path(waypoints)

        return path
    
    async def go_home_callback(self):
        """
        Trigger to move arm to home position.

        :param request: service request object
        :type request: std_msgs.msg.Empty
        :returns: response to service call
        :rtype: bool
        """
        # Plan path to home
        await self.plan_path(goal_joints=self.robot_state.home_joint_state)
        self.node.get_logger().info('Finished moving arm to home position')


    async def exec_path(self, path: MotionPlanRequest = None):
        """
        Execute calculated path.

        :param path: path planned for execution
        :type path: moveit_msgs.msg.MotionPlanRequest
        :returns: result of execution
        :rtype: bool
        """
        if path is None:
            # Error message
            result = 0
        else:
            result = await self.motion_planner.execute_plan(path)
        return result
