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
import pickle

from typing import Dict, List, Optional

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    RobotState,
    RobotTrajectory,
    WorkspaceParameters
)
from moveit_msgs.srv import GetCartesianPath

from object_mover.PlanningScene import PlanningScene as CustomPlanningScene
from object_mover.RobotState import RobotState as CustomRobotState
from object_mover.utils import populate_gripper_constraints, populate_joint_constraints

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Header


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

    def __init__(
        self,
        node: Node,
        robot_state: CustomRobotState,
        planning_scene: CustomPlanningScene,
    ):
        """Initialize the MotionPlanner class."""
        self.node = node
        self.robot_state = robot_state
        self.planning_scene = planning_scene
        self.move_action_client = ActionClient(
            self.node,
            MoveGroup,
            'move_action',
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.saved_plans = {}
        self.saved_configurations = {}
        self.node.action_client = ActionClient(
            self.node,
            MoveGroup,
            'move_action',
            callback_group=(rclpy.callback_groups.MutuallyExclusiveCallbackGroup()),
        )

        if not self.move_action_client.wait_for_server(timeout_sec=10):
            raise RuntimeError('MoveGroup action server not ready')

        # Plan cartesian path needed services and actions

        # Calculate cartesian path service
        self.plan_cart_path_client = self.node.create_client(
            srv_name='/compute_cartesian_path',
            srv_type=GetCartesianPath,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Client for the execuate trajectory action
        self.execute_trajectory_client = ActionClient(
            node=self.node,
            action_name='/execute_trajectory',
            action_type=ExecuteTrajectory,
        )

        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10):
            raise RuntimeError('execute_trajectory client action server not ready')

    async def execute_plan(self, plan: MotionPlanRequest):
        """
        Execute a previously planned motion.

        :param plan: The planned motion to be executed.
        :type plan: moveit_msgs.msg.MotionPlanRequest
        :returns: True if execution is successful, False otherwise.
        :rtype: bool
        """
        move_group_goal = MoveGroup.Goal()
        move_group_goal.request = plan
        move_group_goal.planning_options.planning_scene_diff.is_diff = True
        move_group_goal.planning_options.planning_scene_diff.robot_state = (
            plan.start_state
        )
        world_collision_objects = await self.planning_scene.get_collision_objects()
        move_group_goal.planning_options.planning_scene_diff.world.collision_objects = (
            world_collision_objects
        )
        move_group_goal.planning_options.plan_only = False

        goal_handle = await self.move_action_client.send_goal_async(move_group_goal)
        if not goal_handle.accepted:
            return False
        # print type of goal_handle
        result = await goal_handle.get_result_async()
        return result.result.error_code

    async def plan_joint_path(
        self,
        start_joints: Optional[List[float]],
        goal_joints: Dict[str, float],
        execute: bool = False,
        save_plan: bool = False,
        plan_name: str = 'recent',
    ):
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
        path.workspace_parameters = WorkspaceParameters(
            header=Header(
                stamp=self.node.get_clock().now().to_msg(),
                frame_id='fer_link0'
            ),
            min_corner=Vector3(x=-2.0, y=-2.0, z=0.0),
            max_corner=Vector3(x=2.0, y=2.0, z=2.0)
        )
        path.max_velocity_scaling_factor = 0.1
        path.allowed_planning_time = 30.0
        path.max_acceleration_scaling_factor
        path.start_state.joint_state = JointState()
        path.planner_id = 'move_group'
        if start_joints:
            path.start_state.joint_state.position = start_joints

        else:
            # use the current robot state if the starting joint angles are not provided
            current_state = self.get_current_robot_state()
            path.start_state.joint_state.position = current_state.joint_state.position

        path.goal_constraints = [Constraints()]
        path.group_name = 'fer_manipulator'
        path.goal_constraints[0].joint_constraints = [
            JointConstraint(
                joint_name=joint,
                position=angle,
                tolerance_below=0.0001,
                tolerance_above=0.0001,
                weight=1.0,
            )
            for joint, angle in goal_joints.items()
        ]

        if save_plan:
            self.save_plan(path, plan_name)

        if execute:
            result = await self.execute_plan(path)
            return result

        return path

    async def plan_pose_to_pose(
        self,
        start_pose: Optional[Pose],
        goal_pose: Optional[Pose],
        execute: bool = False,
        save_plan: bool = False,
        plan_name: str = 'recent',
    ):
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
        path.workspace_parameters = WorkspaceParameters(
            header=Header(
                stamp=self.node.get_clock().now().to_msg(),
                frame_id='fer_link0'
            ),
            min_corner=Vector3(x=-2.0, y=-2.0, z=0.0),
            max_corner=Vector3(x=2.0, y=2.0, z=2.0)
        )
        path.max_velocity_scaling_factor = 0.1
        path.allowed_planning_time = 30.0
        path.max_acceleration_scaling_factor = 0.1
        path.planner_id = 'move_group'
        path.group_name = 'fer_manipulator'
        current_state = self.get_current_robot_state()

        if not start_pose:
            path.start_state = current_state
        else:
            start_state_ik_solution = await CustomRobotState.compute_IK(
                self.robot_state, start_pose, 'fer_manipulator'
            )
            path.start_state = start_state_ik_solution.solution

        if not goal_pose.position:
            # Fill out the goal_pose.position with the current position
            # To get the current position, we will need to call the compute_FK function
            fk_solution = await CustomRobotState.compute_FK(
                self.robot_state, ['fer_hand_tcp']
            )
            end_effector_pose = fk_solution[0]
            goal_pose.position = end_effector_pose[0].pose.position

        if not goal_pose.orientation:
            fk_solution = await CustomRobotState.compute_FK(
                self.robot_state, ['fer_hand_tcp']
            )
            end_effector_pose = fk_solution[0]
            goal_pose.orientation = end_effector_pose[0].pose.orientation

        ik_solution = await CustomRobotState.compute_IK(
            self.robot_state, goal_pose, path.start_state.joint_state
        )

        computed_joint_constraints = populate_joint_constraints(ik_solution)
        path.goal_constraints = computed_joint_constraints

        attached_collision_objects = (
            await self.planning_scene.get_attached_collision_objects()
        )
        path.start_state.attached_collision_objects = attached_collision_objects
        if save_plan:
            self.save_plan(path, plan_name)

        if execute:
            result = await self.execute_plan(path)
            return result
        return path

    def _get_cartesian_path_request(
        self,
        waypoints: List[Pose],
        max_step: float = 0.1,
        avoid_collisions: bool = True,
        path_constraints: Constraints = Constraints(),
    ) -> GetCartesianPath.Request:
        """
        Get the Cartesian path request to send to /compute_cartesian_path.

        :param waypoints: A list of poses that define the Cartesian path.
        :type waypoints: List[geometry_msgs.msg.Pose]
        :param max_step: The maximum step between waypoints (?) [m?]
        :type max_step: float
        :param avoid_collisions: Whether to avoid collisions or not.
        :type avoid_collisions: bool
        :param path_constraints: The path's constraints.
        :type path_constraints: moveit_msgs.msg.Constraints
        :returns: The planned motion path request.
        :rtype: moveit_msgs.srv.GetCartesianPath
        """
        request = GetCartesianPath.Request()

        stamp = Time()

        stamp.nanosec = self.node.get_clock().now().nanoseconds

        stamp.sec = int(stamp.nanosec // 1.0e9)

        request.header = Header(stamp=stamp, frame_id='base')
        request.link_name = 'fer_hand_tcp'
        request.group_name = 'fer_arm'
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.waypoints = waypoints

        request.max_step = max_step

        request.avoid_collisions = avoid_collisions

        request.path_constraints = path_constraints

        return request

    async def plan_cartesian_path(
        self,
        waypoints: List[Pose],
        max_step: float = 0.1,
        avoid_collisions: bool = True,
        path_constraints: Constraints = Constraints(),
    ) -> RobotTrajectory:
        """
        Plan a Cartesian path from a sequence of waypoints.

        :param waypoints: A list of poses that define the Cartesian path.
        :type waypoints: List[geometry_msgs.msg.Pose]
        :param max_step: The maximum step between waypoints (?) [m?]
        :type max_step: float
        :param avoid_collisions: Whether to avoid collisions or not.
        :type avoid_collisions: bool
        :param path_constraints: The path's constraints.
        :type path_constraints: moveit_msgs.msg.Constraints
        :returns: The planned motion path request.
        :rtype: moveit_msgs.srv.GetCartesianPath
        """
        path_request = self._get_cartesian_path_request(
            waypoints, max_step, avoid_collisions, path_constraints
        )
        robot_traj = await self.plan_cart_path_client.call_async(path_request)
        robot_traj = robot_traj.solution
        return robot_traj

    def execute_trajectory(self, robot_traj: RobotTrajectory):
        """
        Execute a trajectory using the ExecuteTrajectory action.

        :param robot_traj: The robot trajectory to execute.
        The output from the plan_cartesian_path function.
        :type robot_traj: moveit_msgs.msg.RobotTrajectory
        """
        action = ExecuteTrajectory.Goal()
        action.trajectory = robot_traj
        action.controller_names = ['fer_arm_controller', 'fer_gripper']

        self.execute_trajectory_client.wait_for_server()

        self.execute_trajectory_future = self.execute_trajectory_client.send_goal_async(
            action
        )
        self.execute_trajectory_future.add_done_callback(
            self._execute_trajectory_response_callback
        )

    def _execute_trajectory_response_callback(self, future):
        """
        Handle completion of the ExecuteTrajectory action.

        :param future: The completed future object.
        :type future: Future (?)
        """
        pass
        # response = future.result()

    async def toggle_gripper(self, gripper_state: str, execute: bool = True):
        """
        Toggle the gripper of the arm.

        :returns: The planned motion plan request
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        path = MotionPlanRequest()
        path.allowed_planning_time = 20.0
        path.max_velocity_scaling_factor = 0.1
        path.max_acceleration_scaling_factor = 0.1
        path.group_name = 'hand'
        current_state = self.get_current_robot_state()
        path.start_state = current_state
        path.goal_constraints = populate_gripper_constraints(gripper_state)
        if execute:
            await self.execute_plan(path)
        return path

    async def plan_to_named_configuration(
        self, start_pose: Optional[Pose], named_configuration: str, file_name: str = 'saved_configurations.pkl'):
        """
        Plan a path to a named configuration.

        :param named_configuration: The name of the configuration to plan to.
        :type named_configuration: str
        :returns: The planned motion path request.
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        try:
            with open(file_name, 'rb') as f:
                saved_dict = pickle.load(f)
            goal_constraints = saved_dict.get(named_configuration, None)
            if(goal_constraints is None):
                return None
        except (FileNotFoundError, EOFError): 
            return None 
        path = MotionPlanRequest()
        path.group_name = 'fer_manipulator'
        current_state = self.get_current_robot_state()

        if not start_pose:
            path.start_state = current_state
        else:
            start_state_ik_solution = await CustomRobotState.compute_IK(
                self.robot_state, start_pose, 'fer_manipulator'
            )
            path.start_state = start_state_ik_solution.solution

        path.goal_constraints = goal_constraints
        return path

    def save_configuration(
        self, configuration_name: str, joint_configuration: Dict[str, float], file_name: str = 'saved_configurations.pkl'):
        """
        Save a joint configuration with a name.

        :param configuration_name: The name of the robot configuration
        :param joint_configuration: The goal joint angles
        :type joint_configuration: Dict[str, float]
        """
        saved_joint_constraints = [Constraints()]
        saved_joint_constraints[0].joint_constraints = [
            JointConstraint(
                joint_name=joint,
                position=angle,
                tolerance_below=0.0001,
                tolerance_above=0.0001,
                weight=1.0,
            )
            for joint, angle in joint_configuration.items()
        ]
        try:
            with open(file_name, 'rb') as f:
                saved_dict = pickle.load(f)
        except (FileNotFoundError, EOFError):
            saved_dict = {}

        saved_dict[configuration_name] = joint_configuration
        with open(file_name, 'wb') as f:
            pickle.dump(saved_dict, f)

    def save_plan(self, plan: MotionPlanRequest | RobotTrajectory, plan_name: str, file_name: str = 'saved_plans.pkl'):
        """
        Save a motion plan for future execution.

        :param plan: The motion plan to save.
        :type plan: moveit_msgs.msg.MotionPlanRequest | moveit_msgs.msg.RobotTrajectory
        :param plan_name: The name of the plan to be saved.
        """
        try:
            with open(file_name, 'rb') as f:
                saved_dict = pickle.load(f)
        except (FileNotFoundError, EOFError):
            saved_dict = {}

        saved_dict[plan_name] = plan

        with open(file_name, 'wb') as f:
            pickle.dump(saved_dict, f)

    def inspect_plan(self, plan_name: str, 
                     file_name: str = 'saved_plans.pkl'):
        """
        Inspect a previously saved motion plan.

        :param plan_name: The name of the motion plan to inspect.
        """
        # Find a way to format this better
        try:
            with open(file_name, 'rb') as f:
                saved_dict = pickle.load(f)
            return saved_dict.get(plan_name, None)
        except (FileNotFoundError, EOFError): 
            return None
    def get_current_robot_state(self) -> RobotState:
        """
        Get the current state of the robot.

        :returns: The current state of the robot.
        :rtype: moveit_msgs.msg.RobotState
        """
        return self.robot_state.get_robot_state()
