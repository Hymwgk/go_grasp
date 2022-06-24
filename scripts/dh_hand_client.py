#! /usr/bin/env python

import rospy
import actionlib
from dh_hand_driver.msg import *
from dh_hand_driver.srv import *


def go_position(MotorID, force, position):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    # client = actionlib.SimpleActionClient('actuate_hand', dh_hand_driver.action.ActuateHand)
    client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    # goal = dh_hand_driver.action.ActuateHandGoal()
    goal = ActuateHandGoal()
    goal.MotorID = MotorID
    goal.force = force
    goal.position = position

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


def hand_close(force):
    res = go_position(1, int(force), 0)
    return res


def hand_open():
    res = go_position(1, 50, 95)
    return res


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('actuate_hand_client')
        # result = go_position()
        result = hand_close(50)
        # result = hand_open()
        print("Result:", result.opration_done)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
