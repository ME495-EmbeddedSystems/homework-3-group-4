import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from object_mover.MotionPlanningInerface import MotionPlanningInterface
from object_mover_interfaces.srv import PickPose
from geometry_msgs.msg import Pose
from enum import Enum, auto
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionServer
from moveit_msgs.action import MoveGroup

class State(Enum):
    '''
    Current state of the system (might not be needed)
    '''
    STATE1 = auto()
    STATE2 = auto()
    STATE3 = auto()
    STATE4 = auto()
    STATE5 = auto()
    STATE6 = auto()
    STATE7 = auto()
    STATE8 = auto()


class PickNode(Node):
    def __init__(self):
        super().__init__('pick_node')
        self._cbgroup = MutuallyExclusiveCallbackGroup()
        self.serv = self.create_service(PickPose, 'pick', self.pick_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self._server = ActionServer(self,
                                    MoveGroup,
                                    '/viz/move_action',
                                    self.move_action_callback,
                                    callback_group = self._cbgroup)

        self.mpi = MotionPlanningInterface(self)

    # Maybe need to create custom service type for pick service as we want to be able to load multiple things to it
    async def pick_callback(self, request: PickPose, response):
        # For now I am assuming we are locating pose of object as the input to pick service 
        object_pose = Pose()
        object_pose = request.pick_point
        # Moving arm to right above object
        pose1 = object_pose
        #pose1.position.z = object_pose.position.z + 0.0 #arbitrary number as it has to move above it
        plan = await self.mpi.plan_path(goal_pose = pose1) 
        self.get_logger().info(f"Plan: {plan}")
        await self.mpi.exec_path(path = plan)
        # log plan
        # Opening grippers 

        # Moving arm so object is in between grippers
        # pose2 = object_pose
        # plan = await self.mpi.plan_path(goal_pose = pose2)
        # await self.mpi.exec_path(path = plan)
        # Closing grippers

        # Lifts object slighty off table
        # pose3 = object_pose
        # pose3.position.z = object_pose.position.z + 3.0
        # plan = await self.mpi.plan_path(goal_pose = pose3)
        # await self.mpi.exec_path(path = plan)
        # Attaching box to arm in scene
        #await self.mpi.planning_scene.attach_object('fer_gripper','box')
        # Move arm to other side of obstacle

        # Drop object
        return response

    async def move_action_callback(self, goal_handle):
        self.get_logger().info("Move Action Called. Goal Is")
        # self.get_logger().info(f"{goal_handle.request}") # Do more to format this output nicelynicely
        self.get_logger().info(f"--- End ofrequest dump.----\n")
        self.get_logger().info("Forwarding the action to the move_group action server")
       
        self.get_logger().info("Awaiting the result")
        self.get_logger().info("Interception successful, returning the result")
        return


def pick_entry(args=None):
    '''main function'''
    rclpy.init(args=args)
    node = PickNode()
    rclpy.spin(node)
    rclpy.shutdown()  

if __name__ == '__main__':
    pick_entry()





