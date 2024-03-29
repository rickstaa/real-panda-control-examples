#!/usr/bin/env python3
"""Small node that spins up a dynamic reconfigure server that can be used to change the
joint efforts.
"""
import rospy
from dynamic_reconfigure.server import Server
from real_panda_control_examples.cfg import JointEffortConfig
from std_msgs.msg import Float64
import actionlib
from franka_gripper.msg import MoveAction, MoveGoal
from sensor_msgs.msg import JointState


class JointEffortDynamicReconfigureServer:
    def __init__(self):
        self.srv = Server(JointEffortConfig, self.callback)

        # Create joint effort publishers
        self.arm_joint1_pub = rospy.Publisher(
            "panda_arm_joint1_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint2_pub = rospy.Publisher(
            "panda_arm_joint2_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint3_pub = rospy.Publisher(
            "panda_arm_joint3_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint4_pub = rospy.Publisher(
            "panda_arm_joint4_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint5_pub = rospy.Publisher(
            "panda_arm_joint5_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint6_pub = rospy.Publisher(
            "panda_arm_joint6_effort_controller/command", Float64, queue_size=10
        )
        self.arm_joint7_pub = rospy.Publisher(
            "panda_arm_joint7_effort_controller/command", Float64, queue_size=10
        )
        self.gripper_move_client = actionlib.SimpleActionClient(
            "franka_gripper/move", MoveAction
        )
        self.gripper_move_client.wait_for_server()
        self.arm_pubs = [
            self.arm_joint1_pub,
            self.arm_joint2_pub,
            self.arm_joint3_pub,
            self.arm_joint4_pub,
            self.arm_joint5_pub,
            self.arm_joint6_pub,
            self.arm_joint7_pub,
        ]

    def callback(self, config, level):
        """Dynamic reconfigure callback function.

        Args:
            config (:obj:`dynamic_reconfigure.encoding.Config`): The current dynamic
                reconfigure configuration object.
            level (int): Bitmask that gives information about which parameter has been
                changed.

        Returns:
            :obj:`~dynamic_reconfigure.encoding.Config`: Modified dynamic reconfigure
                configuration object.
        """
        rospy.loginfo(
            (
                "Reconfigure Request: {joint1_effort}, {joint2_effort}, "
                "{joint3_effort} {joint4_effort}, {joint5_effort}, {joint6_effort}, "
                "{joint7_effort}, {width}, {speed}"
            ).format(**config)
        )

        # Set initial joint states
        if level == -1:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            position_dict = dict(zip(joint_states.name, joint_states.position))
            effort_dict = dict(zip(joint_states.name, joint_states.effort))
            set_values = list(effort_dict.values())[:-2]
            set_values.append(list(position_dict.values())[-1] * 2)
            config.update(
                **dict(
                    zip([key for key in config.keys() if key != "groups"], set_values)
                )
            )

        # Write joint positions to controller topics
        if level > -1 and level < 7:
            self.arm_pubs[level].publish(list(config.values())[level])
        elif level in [7, 8]:
            move_goal = MoveGoal(width=config["width"], speed=config["speed"])
            self.gripper_move_client.send_goal(move_goal)
            result = self.gripper_move_client.wait_for_result()
            if not result:
                rospy.logerr("Something went wrong while setting the gripper commands.")
        return config


if __name__ == "__main__":
    rospy.init_node("joint_effort_reconfig_server", anonymous=False)

    effort_dyn_server = JointEffortDynamicReconfigureServer()

    rospy.spin()
