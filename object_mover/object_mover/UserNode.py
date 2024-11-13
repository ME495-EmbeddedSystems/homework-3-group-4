import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from object_mover.MotionPlanner import MotionPlanner
from object_mover.RobotState import RobotState
from moveit_msgs.msg import MotionPlanRequest
from rclpy.action import ActionServer
from moveit_msgs.action import MoveGroup
import object_mover.utils as utils
from object_mover_interfaces.srv import FrankaJointRequest

def main(args=None):
    rclpy.init(args=args)
    node = UserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class UserNode(Node):
    def __init__(self):
        super().__init__('user_node')
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
        self.serv = self.create_service(FrankaJointRequest, 'empty_service', self.empty_service_callback, callback_group=self.cbgroup)


    async def empty_service_callback(self, request: FrankaJointRequest, response):
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
