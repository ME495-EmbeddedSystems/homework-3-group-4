import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from object_mover.MotionPlanningInterface import MotionPlanningInterface
from geometry_msgs.msg import Pose
from enum import Enum, auto

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


class PicKNode(Node):
    def __init__(self):
        super.__init__('pick_node')
        self.serv = self.create_service(Pose, 'pick', self.pick_callback)
        self.mpi = MotionPlanningInterface(self)

    # Maybe need to create custom service type for pick service as we want to be able to load multiple things to it
    async def pick_callback(self, msg):
        # For now I am assuming we are locating pose of object as the input to pick service 
        goal_pose = msg
        pose1 = goal_pose
        # Moving arm to right above object
        pose1.position.z = goal_pose.posiion.z + 3.0 #arbitrary number as it has to move above it
        await self.mpi.plan_path(pose1) 
        await self.mpi.exec_path(self.mpi.path)
        # Opening grippers 

        # Moving arm so object is in between grippers
        pose2 = goal_pose
        await self.mpi.plan_path(pose2)
        await self.mpi.exec_path(pose2)
        # Closing grippers

        # Lifts object slighty off table
        pose3 = goal_pose
        pose3.position.z = goal_pose.position.z + 3.0
        await self.mpi.plan_path(pose3)
        await self.mpi.exec_path(pose3)
        # Attaching box to arm in scene
        await self.mpi.planning_scene.attach_object('fer_gripper','box')
        # Move arm to other side of object

        # Drop object

        




