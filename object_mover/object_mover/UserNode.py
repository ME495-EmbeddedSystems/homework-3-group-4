import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from object_mover.MotionPlanner import MotionPlanner
from object_mover.RobotState import RobotState
from moveit_msgs.msg import MotionPlanRequest
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
        self.cbgroup = MutuallyExclusiveCallbackGroup()

        # create a timer
        self.create_timer(1.0, self.plan_joint_path, callback_group=self.cbgroup)

    async def plan_joint_path(self):
        
        goal_joints = {
            "fer_joint1": 0.0,
            "fer_joint2": -0.785,
            "fer_joint3": 0.0,
            "fer_joint4": -2.355,
            "fer_joint5": 0.0,
            "fer_joint6": 1.57,
            "fer_joint7": 0.0,
            "fer_finger_joint1": 0.0,
            "fer_finger_joint2": 0.0,
        }
        motion_plan_request: MotionPlanRequest = await self.motion_planner.plan_joint_path(start_joints=None, goal_joints=goal_joints)
        await self.motion_planner.execute_plan(motion_plan_request)

if __name__ == '__main__':
    import sys
    main(sys.argv)
