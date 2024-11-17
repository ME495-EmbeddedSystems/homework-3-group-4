import rclpy
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import Pose
import rclpy.node
import rclpy.service
import time
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

        self.collision_object_publisher = self.node.create_publisher(CollisionObject, '/collision_object', 10)
        self.attached_collision_object_publisher = self.node.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)

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
        # box.dimensions is float64[] dimensions
        box.dimensions = dimension
        collision_object.id = name
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        self.scene_response.scene.world.collision_objects.append(collision_object)
        response = await self.apply_scene.call_async(ApplyPlanningScene.Request(scene=self.scene_response.scene))
        return response

    async def remove_box(self, name):
        """
        Remove the specified box from the environment by clearing the entire scene
        and recreating it without the specified object.
        :param name: The name of the object to remove.
        :type name: str
        :return: The response of the service call.
        :rtype: ApplyPlanningScene.Response
        """
        # Fetch the current planning scene
        self.scene_response = await self.get_scene.call_async(GetPlanningScene.Request())

        # Step 1: Clear all collision objects and attached objects
        self.node.get_logger().info("Clearing all collision objects...")

        # Remove non-attached collision objects
        for obj in self.scene_response.scene.world.collision_objects:
            remove_obj = CollisionObject()
            remove_obj.id = obj.id
            remove_obj.operation = CollisionObject.REMOVE
            remove_obj.header.stamp = self.node.get_clock().now().to_msg()
            remove_obj.header.frame_id = obj.header.frame_id
            self.collision_object_publisher.publish(remove_obj)

        # Remove attached objects
        for attached_obj in self.scene_response.scene.robot_state.attached_collision_objects:
            remove_attached_obj = AttachedCollisionObject()
            remove_attached_obj.object.id = attached_obj.object.id
            remove_attached_obj.object.operation = CollisionObject.REMOVE
            remove_attached_obj.link_name = attached_obj.link_name
            remove_attached_obj.object.header.stamp = self.node.get_clock().now().to_msg()
            remove_attached_obj.object.header.frame_id = attached_obj.object.header.frame_id
            self.attached_collision_object_publisher.publish(remove_attached_obj)

        # Allow some time for the changes to take effect
        time.sleep(1)

        # Fetch the updated planning scene to verify changes
        self.scene_response = await self.get_scene.call_async(GetPlanningScene.Request())
        self.node.get_logger().info("Re-adding objects except the one to be removed...")

        # Step 2: Re-add objects except the one to be removed
        for obj_name, obj in self.objects.items():
            if obj_name != name:
                self.collision_object_publisher.publish(obj)

        for obj_name, attached_obj in self.attach_objects.items():
            if obj_name != name:
                self.attached_collision_object_publisher.publish(attached_obj)

        # Remove the object from the internal dictionary if it exists
        if name in self.objects:
            del self.objects[name]
        if name in self.attach_objects:
            del self.attach_objects[name]

        # Verify that the object was removed successfully
        self.scene_response = await self.get_scene.call_async(GetPlanningScene.Request())
        self.node.get_logger().info(f"Objects remaining: {[obj.id for obj in self.scene_response.scene.world.collision_objects]}")

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
        attach_object.link_name = 'fer_hand_tcp'
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
    
