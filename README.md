# Homework 3 (Group 4)

Authors: Ben Benyamin, David Khachatryan, Pushkar Dave, Haodong Wang, Sairam Umakanth  

Reference for using move it
```py
"""Move it request example."""

import moveit_msgs
import std_msgs
import geometry_msgs
import builtin_interfaces
import sensor_msgs
import octomap_msgs

moveit_msgs.action.MoveGroup_Goal(
    request=moveit_msgs.msg.MotionPlanRequest(
        workspace_parameters=moveit_msgs.msg.WorkspaceParameters(
            header=std_msgs.msg.Header(
                stamp=builtin_interfaces.msg.Time(sec=1731364976, nanosec=41770398),
                frame_id="base",
            ),
            min_corner=geometry_msgs.msg.Vector3(x=-1.0, y=-1.0, z=-1.0),
            max_corner=geometry_msgs.msg.Vector3(x=1.0, y=1.0, z=1.0),
        ),
        start_state=moveit_msgs.msg.RobotState(
            joint_state=sensor_msgs.msg.JointState(
                header=std_msgs.msg.Header(
                    stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id="base"
                ),
                name=[
                    "fer_joint1",
                    "fer_joint2",
                    "fer_joint3",
                    "fer_joint4",
                    "fer_joint5",
                    "fer_joint6",
                    "fer_joint7",
                    "fer_finger_joint1",
                    "fer_finger_joint2",
                ],
                position=[
                    0.0,
                    -0.7853981633974483,
                    0.0,
                    -2.356194490192345,
                    0.0,
                    1.5707963267948966,
                    0.7853981633974483,
                    0.0,
                    0.0,
                ],
                velocity=[],
                effort=[],
            ),
            multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(
                header=std_msgs.msg.Header(
                    stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id="base"
                ),
                joint_names=[],
                transforms=[],
                twist=[],
                wrench=[],
            ),
            attached_collision_objects=[],
            is_diff=False,
        ),
        goal_constraints=[
            moveit_msgs.msg.Constraints(
                name="",
                joint_constraints=[
                    moveit_msgs.msg.JointConstraint(
                        joint_name="fer_joint1",
                        position=0.701579856120492,
                        tolerance_above=0.0001,
                        tolerance_below=0.0001,
                        weight=1.0,
                    ),
                    moveit_msgs.msg.JointConstraint(
                        joint_name="fer_joint2",
                        position=0.8214184123406906,
                        tolerance_above=0.0001,
                        tolerance_below=0.0001,
                        weight=1.0,
                    ),
                    moveit_msgs.msg.JointConstraint(
                        joint_name="fer_joint3",
                        position=0.7868758148905707,
                        tolerance_above=0.0001,
                        tolerance_below=0.0001,
                        weight=1.0,
                    ),
                    moveit_msgs.msg.JointConstraint(
                        joint_name="fer_joint4",
                        position=-0.9702014692776181,
                        tolerance_above=0.0001,
                        tolerance_below=0.0001,
                        weight=1.0,
                    ),
                    moveit_msgs.msg.JointConstraint(
                        joint_name="fer_joint5",
                        position=-0.5455567781460792,
                        tolerance_above=0.0001,
                        tolerance_below=0.0001,
                        weight=1.0,
                    ),
                    moveit_msgs.msg.JointConstraint(
                        joint_name="fer_joint6",
                        position=1.6123023279997923,
                        tolerance_above=0.0001,
                        tolerance_below=0.0001,
                        weight=1.0,
                    ),
                    moveit_msgs.msg.JointConstraint(
                        joint_name="fer_joint7",
                        position=2.111524208044342,
                        tolerance_above=0.0001,
                        tolerance_below=0.0001,
                        weight=1.0,
                    ),
                ],
                position_constraints=[],
                orientation_constraints=[],
                visibility_constraints=[],
            )
        ],
        path_constraints=moveit_msgs.msg.Constraints(
            name="",
            joint_constraints=[],
            position_constraints=[],
            orientation_constraints=[],
            visibility_constraints=[],
        ),
        trajectory_constraints=moveit_msgs.msg.TrajectoryConstraints(constraints=[]),
        reference_trajectories=[],
        pipeline_id="ompl",
        planner_id="",
        group_name="fer_arm",
        num_planning_attempts=10,
        allowed_planning_time=5.0,
        max_velocity_scaling_factor=0.1,
        max_acceleration_scaling_factor=0.1,
        cartesian_speed_limited_link="",
        max_cartesian_speed=0.0,
    ),
    planning_options=moveit_msgs.msg.PlanningOptions(
        planning_scene_diff=moveit_msgs.msg.PlanningScene(
            name="",
            robot_state=moveit_msgs.msg.RobotState(
                joint_state=sensor_msgs.msg.JointState(
                    header=std_msgs.msg.Header(
                        stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=""
                    ),
                    name=[],
                    position=[],
                    velocity=[],
                    effort=[],
                ),
                multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(
                    header=std_msgs.msg.Header(
                        stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=""
                    ),
                    joint_names=[],
                    transforms=[],
                    twist=[],
                    wrench=[],
                ),
                attached_collision_objects=[],
                is_diff=True,
            ),
            robot_model_name="",
            fixed_frame_transforms=[],
            allowed_collision_matrix=moveit_msgs.msg.AllowedCollisionMatrix(
                entry_names=[],
                entry_values=[],
                default_entry_names=[],
                default_entry_values=[],
            ),
            link_padding=[],
            link_scale=[],
            object_colors=[],
            world=moveit_msgs.msg.PlanningSceneWorld(
                collision_objects=[],
                octomap=octomap_msgs.msg.OctomapWithPose(
                    header=std_msgs.msg.Header(
                        stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=""
                    ),
                    origin=geometry_msgs.msg.Pose(
                        position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0),
                        orientation=geometry_msgs.msg.Quaternion(
                            x=0.0, y=0.0, z=0.0, w=1.0
                        ),
                    ),
                    octomap=octomap_msgs.msg.Octomap(
                        header=std_msgs.msg.Header(
                            stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0),
                            frame_id="",
                        ),
                        binary=False,
                        id="",
                        resolution=0.0,
                        data=[],
                    ),
                ),
            ),
            is_diff=True,
        ),
        plan_only=True,
        look_around=False,
        look_around_attempts=0,
        max_safe_execution_cost=0.0,
        replan=False,
        replan_attempts=0,
        replan_delay=0.0,
    ),
)

```
