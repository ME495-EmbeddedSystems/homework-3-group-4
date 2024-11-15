import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_srvs.srv import Empty

from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.msg import Pose

from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetCartesianPath

from visualization_msgs.msg import Marker, MarkerArray

from rclpy.action import ActionServer ,ActionClient
from rclpy.action.server import ServerGoalHandle

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from builtin_interfaces.msg import Time

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState as RobotStateMoveit , Constraints ,RobotTrajectory

from moveit_msgs.srv import GetPositionFK
from moveit_msgs.action import ExecuteTrajectory

from  object_mover.RobotState import RobotState

from geometry_msgs.msg import Pose, Point, Quaternion

class InspectMoveit(Node):


    def __init__(self):

        super().__init__('inspector')

        
        qos_profile = QoSProfile(depth=10)

        self.action_server = ActionServer(
            self,
            MoveGroup,
            '/move_action',
            self.server_callback
        )

        self.execute_trajectory_client = ActionClient(
            self,
            action_name= '/execute_trajectory',
            action_type= ExecuteTrajectory
        )

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.fk_client = self.create_client(
            srv_type= GetPositionFK,
            srv_name= '/compute_IK'
        )

        self.joint_states = JointState()

        self.plan_cart_path_client = self.create_client(
            srv_name= '/compute_cartesian_path',
            srv_type= GetCartesianPath,
            qos_profile= qos_profile,
            callback_group= MutuallyExclusiveCallbackGroup()
        )

        self.resp = None

        while not self.plan_cart_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


        self.robot_state = RobotState(self)

        self.robot_traj =  RobotTrajectory()

        self.frequency = 0.5  # Hz

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        self.sent_traj = False

    async def timer_callback(self):

        
        if (self.resp is None):

            self.get_logger().info("Sending request....")
            req = await self.get_cartesian_path_request()
            self.get_logger().info(f'{req}')
            print("-"*100)
            self.resp = self.plan_cart_path_client.call_async(req)
        
        elif (self.resp.done and not self.sent_traj):
            pass
            print(self.resp.result())
            self.robot_traj = self.resp.result().solution
            self.execute_trajectory()

    def server_callback(self, move: ServerGoalHandle):

        self.get_logger().info(f'Request : {move.request}')

        # Assuming MoveGroupResult is what needs to be returned
        result = MoveGroup.Result()
        # Set the result values appropriately, depending on what you're doing
        result.error_code.val = 1  # Example of setting an error code

        # Return the result in the proper format
        return MoveGroup.Result(result=result)


    def joint_state_callback(self,joint_states):
        
        self.joint_states = joint_states


    async def get_cartesian_path_request(self):

        # /execute_trajectory [moveit_msgs/action/ExecuteTrajectory]:
        # https://docs.ros.org/en/humble/p/moveit_msgs/interfaces/action/ExecuteTrajectory.html

        request = GetCartesianPath.Request()

        stamp = Time()

        stamp.nanosec = self.get_clock().now().nanoseconds
        
        stamp.sec = int(stamp.nanosec // 1.0e9)

        request.header = Header(
            stamp = stamp,
            frame_id = "base"
        )

        request.group_name  = "fer_arm"

        robot_state = RobotStateMoveit()

        robot_state.joint_state = self.joint_states

        request.start_state = robot_state

        start_pos = await self.robot_state.compute_FK()

        #Get the Pose element out of the response
        start_pos = start_pos.pose_stamped[0].pose

        position = Point(x=0.4, y=4.0639413383265914e-16, z=0.6972820523028392)
        orientation = Quaternion(x=0.9238795325112868, y=-0.38268343236508945, z=-1.4845532705112254e-16, w=-2.783989563323368e-16)

        # Create the Pose object
        pose = Pose(position=position, orientation=orientation)

        waypoints = [pose]

        request.waypoints = waypoints

        request.max_step = 0.1

        request.avoid_collisions = True

        request.path_constraints = Constraints()

        
        return request
    
    
    async def compute_FK(self):

        req = GetPositionFK.Request()
        req.robot_state.joint_state = self.joint_states

        response =  self.fk_client.call_async(req)
        
        while (not response.done()):
            print("waiting...")

        return response.pose

    def execute_trajectory(self):
        
        action = ExecuteTrajectory.Goal()
        
        action.trajectory = self.robot_traj
        
        action.controller_names = ["fer_arm_controller","fer_gripper"]

        
        self.execute_trajectory_client.wait_for_server()

        self.execute_trajectory_future = self.execute_trajectory_client.send_goal_async(action)
        self.execute_trajectory_future.add_done_callback(self.execute_trajectory_response_callback)

    def execute_trajectory_response_callback(self,future):
        
        response = future.result()
        print("*"*100)
        print(response)
        self.sent_traj = True

def main(args = None):

    rclpy.init(args=args)

    inspector_node = InspectMoveit()

    rclpy.spin(inspector_node)