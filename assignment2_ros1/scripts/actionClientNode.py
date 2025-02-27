#!/usr/bin/env python
"""
.. module:: actionClientNode
   :platform: Unix
   :synopsis: This module contains the code for the ROS1 package of the assignment 2.

.. moduleauthor:: Tinfena Mattia - s7852527@studenti.unige.it

This is the action client node written in Python for the assignment 2 of the research track course.
"""

import rospy
import actionlib
import actionlib.msg
import sys
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from assignment2_ros1.msg import RobotInfo

# Global variable to store the latest feedback
feedbackData = None

def feedbackHandler(feedback):
    """
    Callback to handle feedback from the action server.

    :param feedback: The feedback message received from the action server.
    :type feedback: PlanningFeedback
    """
    global feedbackData
    feedbackData = feedback

def sendGoal(client, xTarget, yTarget):
    """
    Sends a goal to the action server.

    This function sends the target coordinates to the action server and initiates 
    goal tracking. It also sets up the feedback callback function.

    :param client: The action client instance that communicates with the action server.
    :type client: actionlib.SimpleActionClient
    :param xTarget: The X coordinate of the target goal.
    :type xTarget: float
    :param yTarget: The Y coordinate of the target goal.
    :type yTarget: float
    """
    goal = PlanningGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = xTarget
    goal.target_pose.pose.position.y = yTarget
    client.send_goal(goal, feedback_cb=feedbackHandler)
    rospy.loginfo("Goal has been sent to the action server.")

def userInteraction(client):
    """
    Handles user inputs for goal management.

    This function provides a simple interface for the user to interact with the 
    action client. The user can cancel goals, request feedback, or exit the program.

    :param client: The action client instance.
    :type client: actionlib.SimpleActionClient
    :returns: A string indicating the action taken, or "exit" to stop the program.
    :rtype: str
    """
    command = input("Enter 'q' to cancel the goal, 'f' for feedback, or 'e' to exit: ")
    if command.lower() == "q":
        if client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
            rospy.loginfo("Cancelling the current goal.")
            client.cancel_goal()
        else:
            rospy.loginfo("The goal has already been achieved.")
        return "deleted"
    elif command.lower() == "f":
        rospy.loginfo("Requesting feedback...")
        if feedbackData is None:
            rospy.loginfo("No feedback received yet.")
        else:
            rospy.loginfo("Latest feedback: %s", feedbackData)
    elif command.lower() == "e":
        if client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
            rospy.loginfo("Cancelling goal and shutting down.")
            client.cancel_goal()
        else:
            rospy.loginfo("Shutting down.")
        return "exit"
    else:
        rospy.loginfo("Invalid input. Please try again.")

def getCoordinate(prompt):
    """
    Prompts the user to input a valid coordinate.

    This function continually asks for user input until a valid number is entered. 
    It will also exit the program if the user enters 'e'.

    :param prompt: The prompt message to display to the user.
    :type prompt: str
    :returns: The valid coordinate input by the user or "exit" if the user chooses to exit.
    :rtype: float or str
    """
    while True:
        userInput = input(prompt)
        if userInput == "e":
            return "exit"
        try:
            value = float(userInput)
            return value
        except ValueError:
            rospy.logwarn("Invalid input. Please enter a valid number.")

def publishStatus(odomMsg):
    """
    Publishes the robot's current position and velocity.

    This function receives an Odometry message, extracts the robot's position and velocity, 
    and publishes it to the '/robot_status' topic.

    :param odomMsg: The Odometry message containing the robot's position and velocity.
    :type odomMsg: Odometry
    """
    robotStatus = RobotInfo()
    robotStatus.x = odomMsg.pose.pose.position.x
    robotStatus.y = odomMsg.pose.pose.position.y
    robotStatus.velX = odomMsg.twist.twist.linear.x
    robotStatus.velZ = odomMsg.twist.twist.angular.z
    statusPublisher.publish(robotStatus)

if __name__ == '__main__':
    """
    Main entry point for the ROS node.

    This function initializes the ROS node, handles user interaction, and manages the sending 
    of goals to the action server. It keeps the node running until the user exits or an error occurs.
    """
    try:
        rospy.init_node("robotActionClient")
        rospy.sleep(2)
        statusPublisher = rospy.Publisher("/robot_status", RobotInfo, queue_size=10)
        rospy.Subscriber("/odom", Odometry, publishStatus)
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            rate.sleep()  # Request target coordinates from the user
            xTarget = getCoordinate("Enter X coordinate for the goal (enter 'e' to exit): ")
            if xTarget == "exit":
                break
            yTarget = getCoordinate("Enter Y coordinate for the goal (enter 'e' to exit): ")
            if yTarget == "exit":
                break
            
            # Initialize the action client and wait for the server
            actionClient = actionlib.SimpleActionClient('reaching_goal', PlanningAction)
            actionClient.wait_for_server()
            
            sendGoal(actionClient, xTarget, yTarget)
            
            while actionClient.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
                result = userInteraction(actionClient)
                rate.sleep()
                if result == "exit" or result == "deleted":
                    break
            if result == "exit":
                break

        rospy.loginfo("Node terminated successfully.")
    except rospy.ROSInterruptException:
        rospy.logerr("Action client was interrupted.")
        sys.exit(1)
