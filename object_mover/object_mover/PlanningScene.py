# Copyright 2024 David davidkh@u.northwestern.edu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from typing import List

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class PlanningScene:
    """Define the PlanningScene class."""

    def __init__(self, node: Node):
        """Initialize the PlanningScene class."""
        self.node = node
        self.get_scene = self.node.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        while not self.get_scene.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'get_scene service not available, waiting again...'
            )

        self.apply_scene = self.node.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        while not self.apply_scene.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                'apply_scene service not available, waiting again...'
            )
        self.objects = {}
        self.attach_objects = {}
        self.collision_object_publisher = self.node.create_publisher(
            CollisionObject, '/collision_object', 10
        )
        self.attached_collision_object_publisher = self.node.create_publisher(
            AttachedCollisionObject, '/attached_collision_object', 10
        )

    async def add_collision_objects(self, name, position, dimension):
        """
        Add the collision object to the environment.

        :param name: the name of the object
        :type name: str
        :param position:the position of the object
        :type position:tuple
        :dimension:the dimension of the object
        :type dimension:tuple
        :return: the response of the service call
        :rtype: ApplyPlanningScene.Response
        """
        self.scene_response = await self.get_scene.call_async(
            GetPlanningScene.Request()
        )
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'base'
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
        response = await self.apply_scene.call_async(
            ApplyPlanningScene.Request(scene=self.scene_response.scene)
        )
        return response

    async def remove_box(self, name):
        """
        Remove the specified box from the environment and recreate it without the specified object.

        :param name: The name of the object to remove.
        :type name: str
        :return: The response of the service call.
        :rtype: ApplyPlanningScene.Response
        """
        # Fetch the current planning scene
        self.scene_response = await self.get_scene.call_async(
            GetPlanningScene.Request()
        )

        # Remove non-attached collision objects
        for obj in self.scene_response.scene.world.collision_objects:
            remove_obj = CollisionObject()
            remove_obj.id = obj.id
            remove_obj.operation = CollisionObject.REMOVE
            remove_obj.header.stamp = self.node.get_clock().now().to_msg()
            remove_obj.header.frame_id = obj.header.frame_id
            self.collision_object_publisher.publish(remove_obj)

        # Remove attached objects
        for (
            attached_obj
        ) in self.scene_response.scene.robot_state.attached_collision_objects:
            remove_attached_obj = AttachedCollisionObject()
            remove_attached_obj.object.id = attached_obj.object.id
            remove_attached_obj.object.operation = CollisionObject.REMOVE
            remove_attached_obj.link_name = attached_obj.link_name
            remove_attached_obj.object.header.stamp = (
                self.node.get_clock().now().to_msg()
            )
            remove_attached_obj.object.header.frame_id = (
                attached_obj.object.header.frame_id
            )
            self.attached_collision_object_publisher.publish(remove_attached_obj)

        self.scene_response = await self.get_scene.call_async(
            GetPlanningScene.Request()
        )

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
        self.scene_response = await self.get_scene.call_async(
            GetPlanningScene.Request()
        )

    async def attach_object(self, name):
        """
        Attach the box to the end-effector of the robot.

        :param name: the name of the object
        :type name: str
        :return: the response of the service call
        :rtype: ApplyPlanningScene.Response
        """
        # Step 1: Get the planning scene
        current_scene: GetPlanningScene.Response = await self.get_scene.call_async(
            GetPlanningScene.Request()
        )

        # Step 2: Find the object with the given name in the current_scene world collision objects
        collision_objects: List[CollisionObject] = (
            current_scene.scene.world.collision_objects
        )
        # Case 1: Object is found
        collision_object: CollisionObject = None
        for obj in collision_objects:
            if obj.id == name:
                collision_object = collision_objects.pop(collision_objects.index(obj))
                try:
                    del self.objects[name]
                except KeyError:
                    self.node.get_logger().error(
                        f'Object with name {name} not found in the internal dictionary'
                    )
                break

        current_scene.scene.world.collision_objects = []
        for obj in collision_objects:
            if obj.id != name:
                current_scene.scene.world.collision_objects.append(obj)

        if collision_object:
            # Step 3: Attach the object to the end-effector
            attached_object = AttachedCollisionObject()
            attached_object.object = collision_object
            attached_object.link_name = 'fer_hand_tcp'
            attached_object.touch_links = [
                'fer_hand',
                'fer_leftfinger',
                'fer_rightfinger',
            ]
            attached_object.weight = 0.0
            current_scene.scene.robot_state.attached_collision_objects.append(
                attached_object
            )

            # Step 4: Add to the internal dictionary
            self.attach_objects[name] = attached_object

            current_scene.scene.is_diff = True
            # Step 5: Apply the updated scene
            # log the current scene
            update_request = ApplyPlanningScene.Request()
            update_request.scene = current_scene.scene
            response = await self.apply_scene.call_async(update_request)

            current_scene = await self.get_scene.call_async(GetPlanningScene.Request())

            return response
        # Case 2: Object is not found
        else:
            return ApplyPlanningScene.Response(success=False)

    async def detach_object(self, name):
        """
        Detach the box from the robot and add reintroduce the object into the environment.

        :param name: the name of the object
        :type name: str
        :return: the response of the service call
        :rtype: ApplyPlanningScene.Response
        """
        self.scene_response = await self.get_scene.call_async(
            GetPlanningScene.Request()
        )

        try:
            for obj in self.scene_response.scene.robot_state.attached_collision_objects:
                if obj.object.id == name:
                    detach_object = obj
            self.scene_response.scene.world.collision_objects.append(
                detach_object.object
            )
        except BaseException:
            self.node.get_logger().info(
                '-------------The object is not found in the attached objects------------------'
            )
        self.scene_response.scene.robot_state.attached_collision_objects.remove(
            detach_object
        )
        response = await self.apply_scene.call_async(
            ApplyPlanningScene.Request(scene=self.scene_response.scene)
        )
        return response

    async def clear_scene(self):
        """
        Clear the scene.

        :return: the response of the service call
        :rtype: ApplyPlanningScene.Response
        """
        self.scene_response = await self.get_scene.call_async(
            GetPlanningScene.Request()
        )
        self.scene_response.scene.world.collision_objects.clear()
        self.scene_response.scene.robot_state.attached_collision_objects.clear()
        response = await self.apply_scene.call_async(
            ApplyPlanningScene.Request(scene=self.scene_response.scene)
        )
        return response

    async def get_collision_objects(self):
        """
        Return the current collision objects in the scene.

        :return: the list of collision objects
        :rtype: [moveit_msgs.msg.CollisionObject]
        """
        current_scene = await self.get_scene.call_async(GetPlanningScene.Request())
        return current_scene.scene.world.collision_objects

    async def get_attached_collision_objects(self):
        """
        Return the list of attached collision objects.

        :return: the list of attached collision objects in the scene
        :rtype: [moveit_msgs.msg.AttachedCollisionObject]
        """
        current_scene = await self.get_scene.call_async(GetPlanningScene.Request())
        return current_scene.scene.robot_state.attached_collision_objects
