import numpy as np
from moveit_msgs.msg import Constraints, RobotState, JointConstraint

def robot_joints() -> list[str]:
    """
    Return the list of robot joints.
    
    :returns: The list of robot joints.
    :rtype: list[str]
    """
    return [
        'fer_joint1',
        'fer_joint2',
        'fer_joint3',
        'fer_joint4',
        'fer_joint5',
        'fer_joint6',
        'fer_joint7',
        'fer_finger)joint1',
        'fer_finger_joint2',
    ]

def populate_joint_constraints(ik_solution: RobotState) -> list[Constraints]:
    """
    Populate the Constraints msg with JointConstraints

    :param ik_solution: The inverse kinematic solution, consisting of joint names and positions
    :returns: The list with filled JointConstraints
    :rtype: list[Constraints]
    """
    computed_joint_constraints = Constraints()
    
    for index, joint_name in enumerate(ik_solution.solution.joint_state.name):
        position = ik_solution.solution.joint_state.position[index]
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = position
        joint_constraint.tolerance_above = 0.0001
        joint_constraint.tolerance_below = 0.0001
        joint_constraint.weight = 1.0
        computed_joint_constraints.joint_constraints.append(joint_constraint)

    return [computed_joint_constraints]