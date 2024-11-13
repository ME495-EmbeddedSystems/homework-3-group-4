import rclpy
import math
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from object_mover.MotionPlanner import MotionPlanner
from object_mover.RobotState import RobotState
from moveit_msgs.msg import MotionPlanRequest
from rclpy.action import ActionServer
from moveit_msgs.action import MoveGroup
import object_mover.utils as utils


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

        # create a timer
        self.create_timer(1.0, self.plan_joint_path, callback_group=self.cbgroup)

    async def plan_joint_path(self):
        
        goal_joints = {
            "fer_joint1": math.radians(32),
            "fer_joint2": math.radians(-31),
            "fer_joint3": math.radians(-29),
            "fer_joint4": math.radians(-149),
            "fer_joint5": math.radians(-17),
            "fer_joint6": math.radians(119),
            "fer_joint7": math.radians(60),
            "fer_finger_joint1": 0.0,
            "fer_finger_joint2": 0.0,
        }
        motion_plan_request: MotionPlanRequest = await self.motion_planner.plan_joint_path(start_joints=None, goal_joints = {
            "fer_joint1": 0.0,
            "fer_joint2": -0.785,
            "fer_joint3": 0.0,
            "fer_joint4": -2.355,
            "fer_joint5": 0.0,
            "fer_joint6": 1.57,
            "fer_joint7": 0.0,
            "fer_finger_joint1": 0.0,
            "fer_finger_joint2": 0.0,
        })
        await self.motion_planner.execute_plan(motion_plan_request)

    async def move_action_callback(self, goal_handle):
        self.get_logger().info("Move Action Called. Goal Is")
        # self.get_logger().info(f"{goal_handle.request}") # Do more to format this output nicelynicely
        self.get_logger().info(f"--- End ofrequest dump.----\n")
        self.get_logger().info("Forwarding the action to the move_group action server")
       
        self.get_logger().info("Awaiting the result")
        self.get_logger().info("Interception successful, returning the result")
        
        motion_plan_request: MotionPlanRequest = await self.motion_planner.plan_joint_path(start_joints=None, goal_joints = {
            
            
            "fer_joint1": 0.0,
            "fer_joint2": -0.785,
            "fer_joint3": 0.0,
            "fer_joint4": -2.355,
            "fer_joint5": 0.0,
            "fer_joint6": 1.57,
            "fer_joint7": 0.0,
            "fer_finger_joint1": 0.0,
            "fer_finger_joint2": 0.0,
        })
        await self.motion_planner.execute_plan(motion_plan_request)
        return

if __name__ == '__main__':
    import sys
    main(sys.argv)
