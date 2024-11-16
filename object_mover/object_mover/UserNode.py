import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from object_mover.MotionPlanner import MotionPlanner
from object_mover.RobotState import RobotState
from moveit_msgs.msg import MotionPlanRequest
from rclpy.action import ActionServer , ActionClient
from moveit_msgs.action import MoveGroup , ExecuteTrajectory
import object_mover.utils as utils
from object_mover_interfaces.srv import FrankaJointRequest, FrankaPoseRequest, TestPlanningScene, CartesianPathRequest
from object_mover.PlanningScene import PlanningScene
from std_msgs.msg import Empty
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
        self.motion_planner = MotionPlanner(self, robot_state=self.robot_state)
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
        self.cartestian_path_service = self.create_service(CartesianPathRequest, 'test_cartesian_path', self.cartesian_path_callback, callback_group=self.cbgroup)
        self.planning_scene_test = self.create_service(TestPlanningScene, 'test_planning_scene_srv', self.test_planning_scene_callback, callback_group=self.cbgroup)
        self.planning_scene_remove_test = self.create_service(TestPlanningScene, 'test_planning_scene_remove_srv', self.test_planning_scene_remove_callback, callback_group=self.cbgroup)
        self.planning_scene_attach_test = self.create_service(TestPlanningScene, 'test_planning_scene_attach_srv', self.test_planning_scene_attach_callback, callback_group=self.cbgroup)
        self.planning_scene_detach_test = self.create_service(TestPlanningScene, 'test_planning_scene_detach_srv', self.test_planning_scene_detach_callback, callback_group=self.cbgroup)
        self.planning_scene_clear_test = self.create_service(TestPlanningScene, 'test_planning_scene_clear_srv', self.test_planning_scene_clear_callback, callback_group=self.cbgroup)
        self.planning_scene_get_scene = self.create_service(TestPlanningScene, 'test_planning_get_scene_srv', self.test_planning_get_scene_callback, callback_group=self.cbgroup)
        self.plan_test = PlanningScene(self)

    async def test_planning_get_scene_callback(self, request, response):
        await self.plan_test.get_scene()
        response.result = True
        return response

    def test_planning_scene_remove_callback(self, request, response):
        self.plan_test.remove_box('box')
        response.result = True
        return response

    def test_planning_scene_attach_callback(self, request, response):
        link = 'fer_rightfinger'
        self.plan_test.attach_object(link, 'box')

    async def test_planning_scene_clear_callback(self, request, response):
        await self.plan_test.clear_scene()
        response.result = True
        return response

    async def test_planning_scene_callback(self, request, response):
        position = (0.5,0.5,1.0)
        dimenstion = (0.2,0.2,0.2)
        name = 'box'
        await self.plan_test.add_collision_objects(name, position, dimenstion)
        response.result = True
        return response
    
    async def test_planning_scene_remove_callback(self, request, response):
        await self.plan_test.remove_box('box')
        response.result = True
        return response       
    
    async def test_planning_scene_attach_callback(self, request, response):
        link = 'fer_link7'
        await self.plan_test.attach_object(link, 'box')
        response.result = True
        return response


    async def test_planning_scene_detach_callback(self, request, response):
        await self.plan_test.detach_object('box')
        response.result = True
        return response


    async def cartesian_path_callback(self, request: CartesianPathRequest, response):
        self.get_logger().info("Got a request to plan a cartesian path")
        robot_traj = await self.motion_planner.plan_cartesian_path(
            waypoints=request.waypoints,
        )
        
        response.result = True

        self.motion_planner.execute_trajectory(robot_traj)
        
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
