"""Small script to test the gripper controller service."""

import rospy
import actionlib
from franka_gripper.msg import MoveAction, MoveGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommand

if __name__ == "__main__":

    rospy.init_node("fibonacci_client_py")

    # Connect to gripper move service
    move_client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
    move_client.wait_for_server()
    command_client = actionlib.SimpleActionClient(
        "/franka_gripper/gripper_action", GripperCommandAction
    )
    command_client.wait_for_server()

    # Send move goal and wait for result
    move_goal = MoveGoal(width=0.039, speed=2.0)
    move_client.send_goal(move_goal)
    move_client.wait_for_result()
    print(move_client.get_result())

    # Send gripper command and wait for result
    command_goal = GripperCommandGoal(
        command=GripperCommand(position=0.039, max_effort=10.0)
    )
    command_client.send_goal(command_goal)
    command_client.wait_for_result()
    print(command_client.get_result())
    print("test")
