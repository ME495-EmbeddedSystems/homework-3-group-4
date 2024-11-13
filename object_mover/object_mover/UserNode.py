import rclpy
from rclpy.node import Node
from object_mover.MotionPlanner import MotionPlanner
from object_mover.RobotState import RobotState

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

if __name__ == '__main__':
    import sys
    main(sys.argv)
