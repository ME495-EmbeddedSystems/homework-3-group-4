import numpy as np


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