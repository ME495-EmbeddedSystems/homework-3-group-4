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

        self.objects = {}  # Dictionary to store objects with their properties (location, size)
        self.attach_object_dict = {} 
        self.scene_publisher = self.node.create_publisher(MoveitPlanningScene, '/monitored_planning_scene', 10)
        self.scene = MoveitPlanningScene()
        self.scene.robot_state.joint_state.name = [
            'fer_joint1',
            'fer_joint2',
            'fer_joint3',
            'fer_joint4',
            'fer_joint5',
            'fer_joint6',
            'fer_joint7',
            'fer_finger_joint1',
            'fer_finger_joint2'
          ]
        self.scene.robot_state.joint_state.position = [
            0.0,
            -0.7853981633974483,
            0.0,
            -2.356194490192345,
            0.0,
            1.5707963267948966,
            0.7853981633974483,
            0.0,
            0.0
          ]

    # Method to add a box to the scene at a specified location and size
    def add_collision_objects(self, name, position, dimension):
        """
        add the collision object to the environment

        Args:
            the name of the added box, position (tuple), and dimension (tuple)
        
        """
        self.node.get_logger().info('@@@@@@@@@@@@@@@@@@@ add_collision_objects run !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
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

        self.scene_publisher.publish(self.scene)


    def remove_box(self, name):
        """
        remove the box in the environment
        
        Args:
            the box name 

        """

        collision_object = self.objects[name]
        del self.objects[name]
        collision_object.operation = CollisionObject.REMOVE
        # self.node.get_logger().info(f'before update: {self.scene}')
        # self.scene = await self.get_scene.call_async(GetPlanningScene)
        # self.node.get_logger().info(f'after update: {self.scene}')
        self.scene.world.collision_objects.remove(collision_object)
        self.node.get_logger().info('@@@@@@@@@@@@@@@@@@@ prepare to publish attached objects!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        self.scene_publisher.publish(self.scene)


    #Method to attach a collision object to the robot's end-effector
    #Attaching an object requires two operations

    # 1.Removing the original object from the environment   

    # 2.Attaching the object to the robot

    def attach_object(self, linkname, name):
        """
        attach the box to the end-effector 
        Args:
            The linkname that the box attached to, box name

        """
        if name in self.objects:
            collision_object = self.objects[name]
            self.attach_object_dict[name] = self.objects[name]
            self.remove_box(name)

        attach_object = AttachedCollisionObject()
        attach_object.object = collision_object
        attach_object.link_name = linkname
        #3 update scene 
        self.scene.robot_state.attached_collision_objects.append(attach_object)
        self.scene.is_diff= True
        self.scene_publisher.publish(self.scene)
        self.node.get_logger().info(f'after attach: {self.scene}')



    #Method to detach an object from the robot's end-effector
    #Detaching an object from the robot requires two operations

    # 1. Detaching the object from the robot

    # 2. Re-introducing the object into the environment


    def detach_object(self, name):
        """
        detach the box from the robot and add reintroduce the object into the environment
        Args:
            the box name 
        """
        if name in self.attach_object_dict:
            detach_object = self.attach_object_dict[name]
            self.objects[name] = self.attach_object_dict[name].object
            del self.attach_object_dict[name]
        ## get scene here 
        self.scene.robot_state.attached_collision_objects.remove(detach_object)
        self.scene.world.collision_objects.append(detach_object.object)
        self.scene.is_diff= True
        self.scene_publisher.publish(self.scene)
        self.node.get_logger().info(f'after detach: {self.scene}')

