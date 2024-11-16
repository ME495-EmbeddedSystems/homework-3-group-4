import rclpy
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningScene as MoveitPlanningScene
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import Pose
import rclpy.node
import rclpy.service
from shape_msgs.msg import SolidPrimitive

class PlanningScene:
    def __init__(self, node: rclpy.node):
        """
        initialize the PlanningScene class
        """
        self.node = node
        self.get_scene = self.node.create_client(GetPlanningScene, '/get_planning_scene')
        while not self.get_scene.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_scene service not available, waiting again...')

        self.apply_scene = self.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.apply_scene.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('apply_scene service not available, waiting again...')
 
        self.objects = {}  
        self.attach_objects = {}
    async def add_collision_objects(self, name, position, dimension):
        """
        add the collision object to the environment
        :param name: the name of the object
        :type name: str
        :param position:the position of the object
        :type position:tuple
        :dimension:the dimension of the object
        :type dimension:tuple
        :return: the response of the service call
        :rtype: ApplyPlanningScene.Response
        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())        
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base"
        collision_object.id = name

        self.objects[name] = collision_object
        box_pose = Pose()
        box_pose.position.x = position.position.x
        box_pose.position.y = position.position.y
        box_pose.position.z = position.position.z

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimension.data
        collision_object.id = name
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        self.scene_response.scene.world.collision_objects.append(collision_object)
        response = await self.apply_scene.call_async(ApplyPlanningScene.Request(scene=self.scene_response.scene))
        return response

    async def remove_box(self, name):
        """
        remove the box in the environment
        :param name: the name of the object
        :type name: str
        :return: the response of the service call
        :rtype: ApplyPlanningScene.Response
        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())      
        try:
            for object in self.scene_response.scene.world.collision_objects:
                if object.id == name:
                    collision_object = object
            self.scene_response.scene.world.collision_objects.remove(collision_object)

        except:
            self.node.get_logger().info('------------------The object is not in the collision objects------------------')

        try:
            for object_attached in self.scene_response.scene.robot_state.attached_collision_objects:
                if object_attached.object.id == name:
                    object_remove = object_attached
            self.scene_response.scene.robot_state.attached_collision_objects.remove(object_remove)

        except:
            self.node.get_logger().info('------------------The object is not in the attached collision objects------------------')

        response = await self.apply_scene.call_async(ApplyPlanningScene.Request(scene=self.scene_response.scene))
        return response
    
    async def attach_object(self, name):
        """
        attach the box to the end-effector of the robot
        :param name: the name of the object
        :type name: str
        :return: the response of the service call
        :rtype: ApplyPlanningScene.Response
        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())        
        if name in self.objects:
            collision_object = self.objects[name]
        for object in self.scene_response.scene.world.collision_objects:
            if object.id == name:
                self.scene_response.scene.world.collision_objects.remove(object)
                self.attach_objects[name] = object
        del self.objects[name]
        attach_object = AttachedCollisionObject()
        attach_object.object = collision_object
        attach_object.link_name = 'fer_link7'
        self.scene_response.scene.robot_state.attached_collision_objects.append(attach_object)
        response = await self.apply_scene.call_async(ApplyPlanningScene.Request(scene=self.scene_response.scene))
        return response
    
    async def detach_object(self, name):
        """
        detach the box from the robot and add reintroduce the object into the environment
        :param name: the name of the object
        :type name: str
        :return: the response of the service call
        :rtype: ApplyPlanningScene.Response
        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())
        try:        
            for object in self.scene_response.scene.robot_state.attached_collision_objects:
                if object.object.id == name:
                    detach_object = object
            self.scene_response.scene.world.collision_objects.append(detach_object.object)
        except:
            self.node.get_logger().info('-------------The object is not found in the attached objects------------------')
        self.scene_response.scene.robot_state.attached_collision_objects.remove(detach_object)
        response = await self.apply_scene.call_async(ApplyPlanningScene.Request(scene=self.scene_response.scene))
        return response
    
    async def clear_scene(self):
        """
        clear the scene
        :return: the response of the service call
        :rtype: ApplyPlanningScene.Response
        """
        self.scene_response  = await self.get_scene.call_async(GetPlanningScene.Request())     
        self.scene_response.scene.world.collision_objects.clear()
        self.scene_response.scene.robot_state.attached_collision_objects.clear()
        response = await self.apply_scene.call_async(ApplyPlanningScene.Request(scene=self.scene_response.scene))
        return response
    
