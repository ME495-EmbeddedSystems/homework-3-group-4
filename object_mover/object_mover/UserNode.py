import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from object_mover.MotionPlanner import MotionPlanner
from object_mover.RobotState import RobotState
from moveit_msgs.msg import MotionPlanRequest
from rclpy.action import ActionServer , ActionClient
from moveit_msgs.action import MoveGroup , ExecuteTrajectory
import object_mover.utils as utils
from object_mover_interfaces.srv import FrankaJointRequest, FrankaPoseRequest, AddBox, BoxName, CartesianPathRequest, GripperRequest
from object_mover.PlanningScene import PlanningScene
from std_srvs.srv import Empty
from moveit_msgs.srv import GetCartesianPath


def main(args=None):
    rclpy.init(args=args)
    node = UserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class UserNode(Node):
    def __init__(self):
        super().__init__('user_node')
        self.get_logger().info('User Node initialized')
        self.robot_state = RobotState(self)
        self.motion_planner = MotionPlanner(node= self, planning_scene=None, robot_state=self.robot_state)
        self.joints = utils.robot_joints()
        self._cbgroup = MutuallyExclusiveCallbackGroup()
        self._server = ActionServer(self,
                                    MoveGroup,
                                    '/viz/move_action',
                                    self.move_action_callback,
                                    callback_group = self._cbgroup)
        self.cbgroup = MutuallyExclusiveCallbackGroup()

        # create a service that takes an empty request
        self.serv = self.create_service(FrankaJointRequest, 'test_plan_joint_path', self.joint_path_callback, callback_group=self.cbgroup)
        self.pose_serv = self.create_service(FrankaPoseRequest, 'test_plan_pose_to_pose', self.pose_to_pose_callback, callback_group=self.cbgroup)

        self.planning_scene_add_test = self.create_service(AddBox, 'test_planning_scene_add_srv', self.test_planning_scene_add_callback, callback_group=self.cbgroup)
        self.planning_scene_remove_test = self.create_service(BoxName, 'test_planning_scene_remove_srv', self.test_planning_scene_remove_callback, callback_group=self.cbgroup)
        self.planning_scene_attach_test = self.create_service(BoxName, 'test_planning_scene_attach_srv', self.test_planning_scene_attach_callback, callback_group=self.cbgroup)
        self.planning_scene_detach_test = self.create_service(BoxName, 'test_planning_scene_detach_srv', self.test_planning_scene_detach_callback, callback_group=self.cbgroup)
        self.planning_scene_clear_test = self.create_service(Empty, 'test_planning_scene_clear_srv', self.test_planning_scene_clear_callback, callback_group=self.cbgroup)
        self.plan_test = PlanningScene(self)
    
    async def test_planning_scene_clear_callback(self, request, response):
        await self.plan_test.clear_scene()
        return response

    async def test_planning_scene_add_callback(self, request, response):
        await self.plan_test.add_collision_objects(request.name, request.position, request.dimension)
        response.result = True
        return response
    
    async def test_planning_scene_remove_callback(self, request, response):
        await self.plan_test.remove_box(request.name)
        response.result = True
        return response       
    
    async def test_planning_scene_attach_callback(self, request, response):
        await self.plan_test.attach_object(request.name)
        response.result = True
        return response               

    async def test_planning_scene_detach_callback(self, request, response):
        await self.plan_test.detach_object(request.name)
        return response       
        

    async def joint_path_callback(self, request: FrankaJointRequest, response):
        self.get_logger().info("Empty Service Called")
        motion_plan_request: MotionPlanRequest = await self.motion_planner.plan_joint_path(start_joints=None, goal_joints = {
            "fer_joint1": request.joint_angles[0],
            "fer_joint2": request.joint_angles[1],
            "fer_joint3": request.joint_angles[2],
            "fer_joint4": request.joint_angles[3],
            "fer_joint5": request.joint_angles[4],
            "fer_joint6": request.joint_angles[5],
            "fer_joint7": request.joint_angles[6],
            "fer_finger_joint1": request.joint_angles[7],
            "fer_finger_joint2": request.joint_angles[8],
        })
        await self.motion_planner.execute_plan(motion_plan_request)
        return response

    async def pose_to_pose_callback(self, request: FrankaPoseRequest, 
    response):
        self.get_logger().info("Pose service called")
        # self.get_logger().info(f"{request.sample_goal_pose}")
        motion_plan_request: MotionPlanRequest = await self.motion_planner.plan_pose_to_pose(start_pose=None, goal_pose=request.sample_goal_pose)
        await self.motion_planner.execute_plan(motion_plan_request)
        return response

    async def gripper_callback(self, request: GripperRequest, response):
        self.get_logger().info("Gripper service called")
        gripper_resp = await self.motion_planner.toggle_gripper(request.gripper_config)
        return response

    async def move_action_callback(self, goal_handle):
        self.get_logger().info("Move Action Called. Goal Is")
        # self.get_logger().info(f"{goal_handle.request}") # Do more to format this output nicelynicely
        self.get_logger().info(f"--- End ofrequest dump.----\n")
        self.get_logger().info("Forwarding the action to the move_group action server")
       
        self.get_logger().info("Awaiting the result")
        self.get_logger().info("Interception successful, returning the result")
        return

if __name__ == '__main__':
    import sys
    main(sys.argv)
