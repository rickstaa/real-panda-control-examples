#!/usr/bin/env python3
"""Small script to test if we can add a artificial noise to the panda joint commands.

!!!!!!!!!!!!!!!!!!!!!!!!!!IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!
PLEASE BE CARFULL WHEN USING THIS SCRIPT WITH THE REAL ROBOT.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
"""

import rospy
from std_msgs.msg import Float64
import time
import math
import numpy as np
from sensor_msgs.msg import JointState

# Control settings
CONTROL_TYPE = "position"  # NOTE: Options are "effort" and "position"
ACTION_PUB_FREQ = 1000
ACTION_MAGNITUDE = 0.5
ACTION_FREQ = 0.25

# Disturbance settings
DIST_TYPE = None  # NOTE: Options are "impulse", "constant_impulse" and "random"
DISTURBANCE_MAGNITUDE = 0.0
DIST_FREQ = 0.5
DIST_START_TIME = 2.0


def get_disturbance(action):
    """Retrieves a given disturbance based on a disturbance type.

    Args:
        action (np.ndarray): The action the disturbance should be applied to.

    Returns:
        np.ndarray: The disturbance.
    """
    t = time.time()
    if DIST_TYPE == "impulse":
        if math.isclose((t - START_TIME) % (1 / DIST_FREQ), 0.0, abs_tol=0.1):
            return DISTURBANCE_MAGNITUDE * (-np.sign(action))
        else:
            return np.zeros_like(action)
    if DIST_TYPE == "constant_impulse":
        if t >= DIST_START_TIME:
            return DISTURBANCE_MAGNITUDE * (-np.sign(action))
    elif DIST_TYPE == "random":
        return DISTURBANCE_MAGNITUDE * np.random.rand(*action.shape) * -np.sign(action)
    else:
        return np.zeros_like(action)


if __name__ == "__main__":
    rospy.init_node("joint_control_virtual_disturbance_test", anonymous=False)

    # Create required publishers
    publishers = [
        rospy.Publisher(
            f"/panda_arm_joint{idx}_{CONTROL_TYPE}_controller/command",
            Float64,
            queue_size=10,
        )
        for idx in range(1, 8)
    ]

    # Get actual initial joint states
    init_joint_states = rospy.wait_for_message("/joint_states", JointState)
    init_joint_states = dict(
        zip(
            init_joint_states.name,
            init_joint_states.position
            if CONTROL_TYPE == "position"
            else init_joint_states.effort,
        )
    )

    # Publish sinusoid joint command and disturbance
    START_TIME = time.time()
    while True:

        # Create disturbed action
        action = np.array(
            [
                ACTION_MAGNITUDE
                * math.sin(2 * np.pi * ACTION_FREQ * (time.time() - START_TIME)),
            ]
        )
        d = get_disturbance(action)
        d_action = action + d

        # Send control command + disturbance
        publishers[6].publish(init_joint_states["panda_joint7"] + d_action[0])

        # Force publishing rate
        time.sleep(1 / ACTION_PUB_FREQ)
