#!/usr/bin/env python

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
    """Callback to handle feedback from the action server."""
    global feedbackData
    feedbackData = feedback

def sendGoal(client, xTarget, yTarget):
    """Sends a goal to the action server."""
    goal = PlanningGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = xTarget
    goal.target_pose.pose.position.y = yTarget

    client.send_goal(goal, feedback_cb=feedbackHandler)
    rospy.loginfo("Goal has been sent to the action server.")

def userInteraction(client):
    """Handles user inputs for goal management."""
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
    """Prompts the user to input a valid coordinate."""
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
    """Publishes the robot's current position and velocity."""
    robotStatus = RobotInfo()
    robotStatus.x = odomMsg.pose.pose.position.x
    robotStatus.y = odomMsg.pose.pose.position.y
    robotStatus.velX = odomMsg.twist.twist.linear.x
    robotStatus.velZ = odomMsg.twist.twist.angular.z

    statusPublisher.publish(robotStatus)

if __name__ == '__main__':
    try:
        rospy.init_node("robotActionClient")
        rospy.sleep(2)

        statusPublisher = rospy.Publisher("/robot_status", RobotInfo, queue_size=10)
        rospy.Subscriber("/odom", Odometry, publishStatus)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()
            # Request target coordinates from the user
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
