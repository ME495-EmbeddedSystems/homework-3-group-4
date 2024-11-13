from moveit_msgs.srv import GetPositionFK, GetPositionIK
import rclpy.clock
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import rclpy


class RobotState:
    def __init__(self, ros_node):
        self.node = ros_node
        self.joint_state_sub = self.node.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.fk_client = self.node.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        self.joint_state = None

    def joint_state_callback(self, msg):
        self.joint_state = msg

        

    async def compute_IK(self, joint_state: JointState = None):
        if joint_state is None:
            joint_state = self.joint_state
        request = GetPositionIK.Request()
        pose = self.compute_FK(joint_state).pose
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.pose_stamped.header.stamp = self.node.get_cloK().now().to_msg()
        ik_solution = await self.ik_client.call_async(request)
        return ik_solution

    async def compute_FK(self, joint_state: JointState = None):
        if joint_state is None:
            joint_state = self.joint_state
        request = GetPositionFK.Request()
        request.fk_link_names = ['fer_link7']
        request.robot_state.joint_state = joint_state
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result():
            return future.result().solution.pose_stamped
        else:
            return None
        
    def get_robot_state(self):
        robot_state = RobotState()
        robot_state.joint_state = self.joint_state
        return robot_state        
        
