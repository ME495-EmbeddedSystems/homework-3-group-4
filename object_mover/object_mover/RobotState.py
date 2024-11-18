from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from rclpy.node import Node
from moveit_msgs.msg import RobotState as MoveitRobotState
from geometry_msgs.msg import PoseStamped
from typing import List

class RobotState:
    def __init__(self, node: Node):
        self.node = node
        self.joint_state_sub = self.node.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.fk_client = self.node.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        self.joint_state = None

    def joint_state_callback(self, msg):
        self.joint_state = msg

    async def compute_IK(self,  
                        goal_pose: PoseStamped, 
                        joint_state: JointState = None, 
                        group_name: str = 'fer_manipulator'):
        if joint_state is None:
            joint_state = self.joint_state
        request = GetPositionIK.Request()
        request.ik_request.ik_link_name = 'fer_hand_tcp'
        request.ik_request.group_name = group_name
        request.ik_request.robot_state.joint_state = joint_state
        request.ik_request.pose_stamped.pose = goal_pose
        ik_solution = await self.ik_client.call_async(request)
        return ik_solution

    async def compute_FK(self, link_names: List[str],joint_state: JointState = None):
        if joint_state is None:
            joint_state = self.joint_state
        request = GetPositionFK.Request()
        request.fk_link_names = link_names
        # self.node.get_logger().info(link_names)
        request.robot_state.joint_state = joint_state
        fk_solution = await self.fk_client.call_async(request)    
        return fk_solution
        
    def get_robot_state(self) -> MoveitRobotState:
        robot_state = MoveitRobotState()
        robot_state.joint_state = self.joint_state
        return robot_state        
        
