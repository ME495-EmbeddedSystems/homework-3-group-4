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
"""
The pick node is a node that accomplishes the pick and place task of an object by the robotic arm.

It directly interacts with MotionPlanningInterface to access the functions of the objects of
MotionPlanner, RobotState and PlanningScene incorporated within the MotionPlanningInterface class.
"""

import copy

from geometry_msgs.msg import Pose
from object_mover.MotionPlanningInterface import MotionPlanningInterface
from object_mover_interfaces.srv import PickPose
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty


class PickNode(Node):
    """
    Interact with the MotionPlanningInterface to pick and place an object.

    Services:
    - pick: pick an object
    - go_home: move arm to home position
    """

    def __init__(self):
        super().__init__('pick_node')
        self.serv = self.create_service(
            PickPose,
            'pick',
            self.pick_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.go_home = self.create_service(
            Empty,
            'go_home',
            self.go_home_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.mpi = MotionPlanningInterface(self)

    async def pick_callback(self, request: PickPose, response):
        """
        Trigger to pick object.

        :param request: service request object
        :type request: object_mover_interfaces.srv.PickPose
        :returns: response to service call
        :rtype: bool
        """
        # Add box to the scene
        box_pose = Pose()
        box_pose.position.x = 0.59
        box_pose.position.y = 0.13
        box_pose.position.z = 0.1378
        await self.mpi.planning_scene.add_collision_objects(
            'box', box_pose, [0.05, 0.05, 0.05]
        )

        # Add wall to the scene
        wall_pose = Pose()
        wall_pose.position.x = 0.5
        wall_pose.position.y = 0.3
        wall_pose.position.z = 0.0
        await self.mpi.planning_scene.add_collision_objects(
            'wall', wall_pose, [0.05, 0.05, 0.5]
        )

        # Plan path to pick object
        object_pose = Pose()
        object_pose = request.pick_point
        pose1 = copy.deepcopy(object_pose)

        # Step 1: Move arm right above object
        pose1.position.z = object_pose.position.z + 0.2
        traj1 = await self.mpi.plan_path(goal_pose=pose1)
        _ = await self.mpi.exec_path(traj1)
        self.get_logger().info('Step 1: Finished moving arm above object')

        # Step 2: Open Grippers
        await self.mpi.motion_planner.toggle_gripper('open')
        self.get_logger().info('Step 2: Finished opening grippers')

        # Step 3: Move arm to object
        pose2 = object_pose
        traj3 = await self.mpi.plan_path(goal_pose=pose2)
        _ = await self.mpi.exec_path(traj3)
        self.get_logger().info('Step 3: Finished moving arm to object')
        self.mpi.motion_planner.save_plan(plan = traj3, plan_name='plan_3')
        self.get_logger().info(f'Trajectory3: {self.mpi.motion_planner.inspect_plan(plan_name='plan_3')}')

        # Step 4: Closing grippers
        await self.mpi.motion_planner.toggle_gripper('close')
        self.get_logger().info('Step 4: Finished closing grippers')

        # Step 5: Attaching box to arm in scene
        await self.mpi.planning_scene.attach_object('box')
        self.get_logger().info('Step 5: Finished attaching box to arm')

        # Step 6: Move arm up
        pose3 = object_pose
        pose3.position.z = object_pose.position.z + 0.3
        traj6 = await self.mpi.plan_path(goal_pose=pose3)
        _ = await self.mpi.exec_path(traj6)
        self.get_logger().info('Step 6: Finished moving arm up')

        # Step 7: Move arm to other side of obstacle
        pose4 = object_pose
        pose4.position.y = object_pose.position.y + 0.3
        traj7 = await self.mpi.plan_path(goal_pose=pose4)
        _ = await self.mpi.exec_path(traj7)
        self.get_logger().info('Step 7: Finished moving arm to other side of obstacle')

        # Step 8 Drop object
        await self.mpi.motion_planner.toggle_gripper('open')
        self.get_logger().info('Step 8: Finished dropping object')

        # Step 9: Detach object from arm
        await self.mpi.planning_scene.detach_object('box')

        return response

    async def go_home_callback(self, request: Empty, response):
        """
        Trigger to move arm to home position.

        :param request: service request object
        :type request: std_msgs.msg.Empty
        :returns: response to service call
        :rtype: bool
        """
        # Plan path to home
        await self.mpi.plan_path(goal_joints=self.mpi.robot_state.home_joint_state)
        self.get_logger().info('Finished moving arm to home position')
        self.get_logger().info(f'{response}')
        return response


def pick_entry(args=None):
    """Start the pick node."""
    rclpy.init(args=args)
    node = PickNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    pick_entry()
