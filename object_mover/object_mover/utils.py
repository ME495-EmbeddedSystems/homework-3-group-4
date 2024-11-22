from moveit_msgs.msg import Constraints, JointConstraint, RobotState


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
        'fer_finger_joint1',
        'fer_finger_joint2'
    ]

def home_joint_positions() -> list[float]:
    """
    Return the list of home joint positions.

    :returns: The list of home joint positions.
    """
    return [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483, 0.0, 0.0]


def populate_joint_constraints(ik_solution: RobotState) -> list[Constraints]:
    """
    Populate the Constraints msg with JointConstraints.

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


def populate_gripper_constraints(gripper_configuration: str) -> list[Constraints]:
    """Populate the MotionPlanRequest.goal_constraints for opening and closing the gripper."""
    goal_constraints = [Constraints()]
    if gripper_configuration == 'open':
        goal_constraints[0].joint_constraints = [
            JointConstraint(
                joint_name='fer_finger_joint1',
                position=0.035,
                tolerance_above=0.0001,
                tolerance_below=0.0001,
                weight=1.0
            ),
            JointConstraint(
                joint_name='fer_finger_joint2',
                position=0.035,
                tolerance_above=0.0001,
                tolerance_below=0.0001,
                weight=1.0
            )
        ]
    if gripper_configuration == 'close':
        goal_constraints[0].joint_constraints = [
            JointConstraint(
                joint_name='fer_finger_joint1',
                position=0.000,
                tolerance_above=0.0001,
                tolerance_below=0.0001,
                weight=1.0
            ),
            JointConstraint(
                joint_name='fer_finger_joint2',
                position=0.000,
                tolerance_above=0.0001,
                tolerance_below=0.0001,
                weight=1.0
            )
        ]
    return goal_constraints
