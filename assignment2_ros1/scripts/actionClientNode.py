#!/usr/bin/env python

import rospy
import actionlib
import actionlib.msg
import sys
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal, PlanningFeedback
from assignment2_ros1.msg import RobotInfo
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from example_interfaces.srv import Empty, Float64

last_target_x = None
last_target_y = None

feedbackData = None

def feedbackHandler(feedback):
    """Callback to handle feedback from the action server."""
    global feedbackData
    feedbackData = feedback

def feedbackSubscriberCallback(feedbackMsg):
    """Callback function for the feedback subscriber."""
    if feedbackMsg.stat == "Target reached!":  
        rospy.loginfo("reached")

def clbk_laser(msg):
    regions_ = {
        'right':  min(msg.ranges[0:143] or [10]),  
        'fright': min(msg.ranges[144:287] or [10]),
        'front':  min(msg.ranges[288:431] or [10]),
        'fleft':  min(msg.ranges[432:575] or [10]),
        'left':   min(msg.ranges[576:719] or [10]),
    }

    min_distance = min(regions_['right'], regions_['fright'], regions_['front'], regions_['fleft'], regions_['left'])

    warning_msg = Bool()
    warning_msg.data = min_distance < 1.0  
    warning_pub.publish(warning_msg)

    rospy.loginfo("Warning: Closest obstacle at %.2f meters", min_distance)


def calculate_distance(x1, y1, x2, y2):
    """Calculates the Euclidean distance between two points."""
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5

def handle_get_distance(req):
    """Service handler to calculate distance from the last target."""
    if last_target_x is None or last_target_y is None:
        rospy.loginfo("No target has been set yet.")
        return Float64(data=0.0) 

    current_position = rospy.wait_for_message('/odom', Odometry) 
    current_x = current_position.pose.pose.position.x
    current_y = current_position.pose.pose.position.y

    distance = calculate_distance(current_x, current_y, last_target_x, last_target_y)
    rospy.loginfo("Distance to last target: %.2f meters", distance)

    return Float64(data=distance)

def sendGoal(client, xTarget, yTarget):
    """Sends a goal to the action server."""
    global last_target_x, last_target_y
    goal = PlanningGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = xTarget
    goal.target_pose.pose.position.y = yTarget


    last_target_x = xTarget
    last_target_y = yTarget

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

        rospy.Subscriber("/reaching_goal/feedback", assignment_2_2024.msg.PlanningFeedback, feedbackSubscriberCallback)

        rospy.Subscriber("/scan", LaserScan, clbk_laser)

        warning_pub = rospy.Publisher('/warning', Bool, queue_size=10)

        rate = rospy.Rate(10)

        rospy.Service('get_distance_to_target', Empty, handle_get_distance)

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
