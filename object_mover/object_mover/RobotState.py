import sys
from moveit_msgs.srv import GetPositionFK, GetPositionIK
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import rclpy.clock
from sensor_msgs.msg import JointState
import rclpy

class RobotState:
    def __init__(self, ros_node):
        self.node = ros_node
        self.joint_state_sub = self.node.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.FKClient = self.create_client(GetPositionFK, '/compute_FK')
        self.IKClient = self.create_client(GetPositionIK, '/compute_IK')
        self.joint_state = None

    def joint_state_callback(self, msg):
            self.joint_state = msg

    def compute_IK(self, joint_state: JointState = None):    
        if joint_state is None:
            joint_state = self.joint_state
        request = GetPositionIK.Request()
        pose = self.compute_FK(joint_state).pose
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.pose_stamped.header.stamp = self.node.get_cloK().now().to_msg()
        future = self.IKClient.call_async(request)
        rclpy.spin_until_future_complete(self.node,future)
        if future.result():
            return future.result().solution.joint_state
        else:
            return None
        
    def compute_FK(self, joint_state: JointState = None):
        if joint_state is None:
             joint_state = self.joint_state
        request = GetPositionFK.Request()
        request.robot_state.joint_state = joint_state
        future = self.FKClient.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result():
            return future.result().solution.pose_stamped
        else:
            return None
