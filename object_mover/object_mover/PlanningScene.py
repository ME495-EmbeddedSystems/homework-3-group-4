import rclpy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene as MoveitPlanningScene
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import Pose
import rclpy.node
import rclpy.service
from shape_msgs.msg import SolidPrimitive

class PlanningScene:
    def __init__(self, node: rclpy.node):
        self.node = node
        self.get_scene = self.node.create_client(GetPlanningScene, '/get_planning_scene')
        while not self.get_scene.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_scene service not available, waiting again...')
 
        self.objects = {}  
        self.scene_publisher = self.node.create_publisher(MoveitPlanningScene, '/planning_scene', 10)

    async def add_collision_objects(self, name, position, dimension):
        """
        add the collision object to the environment

        Args:
            the name of the added box, position (tuple), and dimension (tuple)
        
        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())        
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
        collision_object.id = name
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        self.scene_response.scene.world.collision_objects.append(collision_object)
        self.scene_publisher.publish(self.scene_response.scene)

    async def remove_box(self, name):
        """
        remove the box in the environment
        
        Args:
            the box name 

        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())      
        for object in self.scene_response.scene.world.collision_objects:
            if object.id == name:
                collision_object = object

        self.scene_response.scene.world.collision_objects.remove(collision_object)
        self.scene_publisher.publish(self.scene_response.scene)

    async def attach_object(self, linkname, name):
        """
        attach the box to the end-effector 
        Args:
            The linkname that the box attached to, box name

        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())        
        if name in self.objects:
            collision_object = self.objects[name]
        attach_object = AttachedCollisionObject()
        attach_object.object = collision_object
        attach_object.link_name = linkname
        self.scene_response.scene.robot_state.attached_collision_objects.append(attach_object)
        self.scene_publisher.publish(self.scene_response.scene)

    async def detach_object(self, name):
        """
        detach the box from the robot and add reintroduce the object into the environment
        Args:
            the box name 
        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())        
        for object in self.scene_response.scene.robot_state.attached_collision_objects:
            if object.object.id == name:
                detach_object = object
        self.scene_response.scene.robot_state.attached_collision_objects.remove(detach_object)
        self.scene_publisher.publish(self.scene_response.scene)

    async def clear_scene(self):
        """
        clear the scene
        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())     
        self.scene_response.scene.world.collision_objects.clear()
        self.scene_response.scene.robot_state.attached_collision_objects.clear()
        self.scene_publisher.publish(self.scene_response.scene)
