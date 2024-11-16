import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from object_mover.MotionPlanningInerface import MotionPlanningInterface
from object_mover_interfaces.srv import PickPose
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


class PickNode(Node):
    def __init__(self):
        super.__init__('pick_node')
        self.serv = self.create_service(PickPose, 'pick', self.pick_callback)
        self.mpi = MotionPlanningInterface(self)

    # Maybe need to create custom service type for pick service as we want to be able to load multiple things to it
    async def pick_callback(self, request, response):
        # For now I am assuming we are locating pose of object as the input to pick service 
        object_pose = Pose()
        object_pose.position = request
        object_pose.orientation = 0.0
        # Moving arm to right above object
        pose1 = object_pose
        pose1.position.z = object_pose.position.z + 3.0 #arbitrary number as it has to move above it
        await self.mpi.plan_path(goal_pose = pose1) 
        await self.mpi.exec_path(path = self.mpi.path)
        # Opening grippers 

        # Moving arm so object is in between grippers
        pose2 = object_pose
        await self.mpi.plan_path(goal_pose = pose2)
        await self.mpi.exec_path(path = pose2)
        # Closing grippers

        # Lifts object slighty off table
        pose3 = object_pose
        pose3.position.z = object_pose.position.z + 3.0
        await self.mpi.plan_path(goal_pose = pose3)
        await self.mpi.exec_path(path = pose3)
        # Attaching box to arm in scene
        await self.mpi.planning_scene.attach_object('fer_gripper','box')
        # Move arm to other side of obstacle

        # Drop object
        return response
    

def pick_entry(args=None):
    '''main function'''
    rclpy.init(args=args)
    node = PickNode()
    rclpy.spin(node)
    rclpy.shutdown()  

if __name__ == '__main__':
    pick_entry()





