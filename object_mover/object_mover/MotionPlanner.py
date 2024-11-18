import rclpy
from rclpy.action import ActionClient
import rclpy.callback_groups
from rclpy.node import Node
from moveit_msgs.action import MoveGroup , ExecuteTrajectory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, Constraints, MotionPlanRequest, JointConstraint, RobotTrajectory, AllowedCollisionMatrix, AllowedCollisionEntry, AttachedCollisionObject
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from typing import Optional, List, Dict
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from object_mover.RobotState import RobotState as CustomRobotState
from object_mover.PlanningScene import PlanningScene as CustomPlanningScene
from object_mover.utils import populate_joint_constraints, populate_gripper_constraints

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

    def __init__(self, node: Node, robot_state: CustomRobotState, planning_scene: CustomPlanningScene):
        """Initialize the MotionPlanner class."""
        self.node = node
        self.robot_state = robot_state
        self.planning_scene = planning_scene
        self.move_action_client = ActionClient(self.node, MoveGroup, 'move_action', callback_group=MutuallyExclusiveCallbackGroup()) 
        self.saved_plans = {}
        self.saved_configurations = {}
        self.node.action_client = ActionClient(self.node, MoveGroup, 'move_action', callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()) 

        if not self.move_action_client.wait_for_server(timeout_sec=10):
            raise RuntimeError('MoveGroup action server not ready')

        ### Plan cartesian path needed services and actions

        # Calculate cartesian path service
        self.plan_cart_path_client = self.node.create_client(
            srv_name= '/compute_cartesian_path',
            srv_type= GetCartesianPath,
            callback_group= MutuallyExclusiveCallbackGroup()
        )

        #Client for the execuate trajectory action
        self.execute_trajectory_client = ActionClient(
            node= self.node,
            action_name= '/execute_trajectory',
            action_type= ExecuteTrajectory
        )

        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10):
            raise RuntimeError('execute_trajectory client action server not ready')

    async def execute_plan(self, plan: MotionPlanRequest, box_attached: bool = False) -> bool:
        """
        Execute a previously planned motion.

        :param plan: The planned motion to be executed.
        :type plan: moveit_msgs.msg.MotionPlanRequest
        :returns: True if execution is successful, False otherwise.
        :rtype: bool
        """
        move_group_goal = MoveGroup.Goal()
        move_group_goal.request = plan
        move_group_goal.planning_options.plan_only = False

        goal_handle = await self.move_action_client.send_goal_async(move_group_goal)
        if not goal_handle.accepted:
            return False

        # print type of goal_handle
        result = await goal_handle.get_result_async()
        return result.result.error_code == 0

    async def plan_joint_path(self, start_joints: Optional[List[float]], goal_joints: Dict[str, float], execute: bool = False, save_plan: bool = False, plan_name: str = 'recent') -> MotionPlanRequest: # noqa 501
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
        path.max_velocity_scaling_factor = 0.5
        path.start_state.joint_state = JointState()

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
                weight=1.0
            )
            for joint, angle in goal_joints.items()
        ]
        
        if save_plan:
            self.save_plan(path, plan_name)
  
        if execute:
            self.execute_plan(path)

        return path

    async def plan_pose_to_pose(self, start_pose: Optional[Pose], goal_pose: Optional[Pose], execute: bool = False, save_plan: bool = False, plan_name: str = 'recent', box_attached: bool = False):
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
        path.max_velocity_scaling_factor = 0.5
        path.group_name = 'fer_manipulator'
        current_state = self.get_current_robot_state()

        if not start_pose:
            path.start_state = current_state
        else:
            start_state_ik_solution = await CustomRobotState.compute_IK(self.robot_state, start_pose, 'fer_manipulator')
            path.start_state = start_state_ik_solution.solution 

        if not goal_pose.position: 
            # Fill out the goal_pose.position with the current position
            # To get the current position, we will need to call the compute_FK function
            fk_solution = await CustomRobotState.compute_FK(self.robot_state,['fer_hand_tcp'])
            end_effector_pose = fk_solution[0]
            goal_pose.position = end_effector_pose[0].pose.position

        if not goal_pose.orientation:
            fk_solution = await CustomRobotState.compute_FK(self.robot_state,['fer_hand_tcp'])
            end_effector_pose = fk_solution[0]
            goal_pose.orientation = end_effector_pose[0].pose.orientation

        ik_solution = await CustomRobotState.compute_IK(self.robot_state,goal_pose, path.start_state.joint_state) 

        computed_joint_constraints = populate_joint_constraints(ik_solution)
        path.goal_constraints = computed_joint_constraints

        # self.node.get_logger().info(f"{await self.planning_scene.get_collision_objects()}")
        # if box_attached:
        #     attached_object = AttachedCollisionObject()
        #     # this is not the correct way to do this
        #     attached_object.link_name = 'fer_hand_tcp'
        #     collision_object_list = await self.planning_scene.get_collision_objects()
        #     attached_object.object = collision_object_list[0]
        #     attached_object.touch_links = ['base', 'fer_hand' 'fer_link0', 'fer_link1', 'fer_link2', 'fer_link3', 'fer_link4', 'fer_link5', 'fer_link6', 'fer_link7', 'fer_link8', 'fer_leftfinger', 'fer_rightfinger']
        #     path.start_state.attached_collision_objects = [attached_object]

        if save_plan:
            self.save_plan(path, plan_name)
            
        if execute:
            self.execute_plan(path, box_attached)
        return path


    def _get_cartesian_path_request(self, waypoints: List[Pose] , max_step : float = 0.1 , avoid_collisions: bool = True, path_constraints: Constraints = Constraints()) -> GetCartesianPath.Request:
        """
        Gets the Cartesian path request to send to /compute_cartesian_path.

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

        #Time stamp of the request
        stamp = Time()

        stamp.nanosec = self.node.get_clock().now().nanoseconds
        
        stamp.sec = int(stamp.nanosec // 1.0e9)

        request.header = Header(
            stamp = stamp,
            frame_id = "base"
        )

        request.group_name  = "fer_arm"

        #Load the waypoints
        
        request.waypoints = waypoints

        request.max_step = max_step

        request.avoid_collisions = avoid_collisions

        request.path_constraints = path_constraints

        return request
    
    async def plan_cartesian_path(self, waypoints: List[Pose] , max_step : float = 0.1 , avoid_collisions: bool = True, path_constraints: Constraints = Constraints()) -> RobotTrajectory:
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
        path_request = self._get_cartesian_path_request(waypoints,max_step,avoid_collisions,path_constraints)
        robot_traj = await self.plan_cart_path_client.call_async(path_request)
        robot_traj = robot_traj.solution
        return robot_traj
    
    def execute_trajectory(self, robot_traj : RobotTrajectory):
        """
        Execute a trajectory using the ExecuteTrajectory action.
        :param robot_traj: The robot trajectory to execute. The output from the plan_cartesian_path function.
        :type robot_traj: moveit_msgs.msg.RobotTrajectory
        """
        action = ExecuteTrajectory.Goal()
        action.trajectory = robot_traj
        action.controller_names = ["fer_arm_controller","fer_gripper"]

        self.execute_trajectory_client.wait_for_server()

        self.execute_trajectory_future = self.execute_trajectory_client.send_goal_async(action)
        self.execute_trajectory_future.add_done_callback(self._execute_trajectory_response_callback)

    def _execute_trajectory_response_callback(self,future):
        """
        Execute trajectory response callback function. Called after the ExecuteTrajectory action future is done.
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
        path.group_name = 'hand'
        current_state = self.get_current_robot_state()
        path.start_state = current_state
        path.goal_constraints = populate_gripper_constraints(gripper_state)
        if execute:
            await self.execute_plan(path)
        return path


    async def plan_to_named_configuration(self, start_pose: Optional[Pose],named_configuration: str):
        """
        Plan a path to a named configuration.

        :param named_configuration: The name of the configuration to plan to.
        :type named_configuration: str
        :returns: The planned motion path request.
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        goal_constraints = self.saved_configurations[named_configuration]
        path = MotionPlanRequest()
        path.group_name = 'fer_manipulator'
        current_state = self.get_current_robot_state()

        if not start_pose:
            path.start_state = current_state
        else:
            start_state_ik_solution = await CustomRobotState.compute_IK(self.robot_state, start_pose, 'fer_manipulator')
            path.start_state = start_state_ik_solution.solution

        path.goal_constraints = goal_constraints
        return path


    def save_configuration(self, configuration_name: str, joint_configuration: Dict[str, float]):
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
                weight=1.0
            )
            for joint, angle in joint_configuration.items()
        ]
        self.saved_configurations[configuration_name] = saved_joint_constraints


    def save_plan(self, plan: MotionPlanRequest | RobotTrajectory, plan_name: str):
        """
        Save a motion plan for future execution.

        :param plan: The motion plan to save.
        :type plan: moveit_msgs.msg.MotionPlanRequest | moveit_msgs.msg.RobotTrajectory
        :param plan_name: The name of the plan to be saved.
        """
        self.saved_plans[plan_name] = plan

    def inspect_plan(self, plan_name: str):
        """
        Inspect a previously saved motion plan.

        :param plan_name: The name of the motion plan to inspect.
        """
        # Find a way to format this better
        print(self.saved_plans[plan_name])

    def get_current_robot_state(self) -> RobotState:
        """
        Get the current state of the robot.

        :returns: The current state of the robot.
        :rtype: moveit_msgs.msg.RobotState
        """
        return self.robot_state.get_robot_state()
