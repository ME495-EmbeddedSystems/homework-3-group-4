import rclpy
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene as MoveitPlanningScene
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import Pose
import rclpy.service
from shape_msgs.msg import SolidPrimitive

class PlanningScene:
    
    def __init__(self, node: rclpy.Node):
        rclpy.init()
        self.node = node
        self.client = self.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if self.client.wait_for_server(timeout_sec=10):
            raise RuntimeError('MoveGroup /apply_planning_scene service not ready')

        self.objects = {}  # Dictionary to store objects with their properties (location, size)
        self.attach_object = {} 
        self.scene = MoveitPlanningScene()

    # Method to add a box to the scene at a specified location and size
    async def add_collision_objects(self, name, position, dimension):
        """
        add the collision object to the environment

        Args:
            the name of the added box, position (tuple), and dimension (tuple)
        
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base"
        collision_object.id = name

        self.objects[name] = collision_object
        box_pose = Pose()
        box_pose.position.x = position[0]
        box_pose.position.y = position[1]
        box_pose.position.z = position[2]

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimension

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        self.scene.world.collision_objects.append(collision_object)
        await self.client.call_async(self.scene)

    async def remove_box(self, name):
        """
        remove the box in the environment
        
        Args:
            the box name 

        """

        collision_object = self.objects[name]
        del self.objects[name]

        collision_object.operation = CollisionObject.REMOVE
        ## get scene 
        self.scene.world.collision_objects.remove(collision_object)

        await self.client.call_async(self.scene)


    #Method to attach a collision object to the robot's end-effector
    #Attaching an object requires two operations

    # 1.Removing the original object from the environment   

    # 2.Attaching the object to the robot

    async def attach_object(self, linkname, name):
        """
        attach the box to the end-effector 
        Args:
            The linkname that the box attached to, box name

        """

        if name in self.objects:
            collision_object = self.objects[name]
            self.attach_object[name] = self.objects[name]
            del self.objects[name]
            self.remove_box(name)
        attach_object = AttachedCollisionObject()
        attach_object.object = collision_object
        attach_object.link_name = linkname
        #3 update scene 
        self.scene.world.attached_collision_objects.append(attach_object)
        await self.client.call_async(self.scene)

    #Method to detach an object from the robot's end-effector
    #Detaching an object from the robot requires two operations

    # 1. Detaching the object from the robot

    # 2. Re-introducing the object into the environment


    async def detach_object(self, name):
        """
        detach the box from the robot and add reintroduce the object into the environment
        Args:
            the box name 
        """
        if name in self.attached_objects:
            self.objects[name] = self.detach_object[name]
            detach_object = self.attached_objects[name]
            del self.attached_objects[name]
        ## get scene here 
        self.scene.world.attached_collision_objects.remove(detach_object)
        self.scene.world.collision_objects.append(detach_object.object)
        await self.client.call_async(self.scene)

