#!/usr/bin/env python3

"""
.. module:: ServiceNode
   :platform: Unix
   :synopsis: This module contains the code for the ROS1 package of the assignment 2.

.. moduleauthor:: Tinfena Mattia - s7852527@studenti.unige.it

This is the service node written in Python for the assignment 2 of the research track course.
"""

import rospy
import sys
from assignment2_ros1.srv import lastTarget, lastTargetResponse  # Import custom service message

def handleTargetRequest(request):
    """
    Callback function for the service that handles requests for the last target coordinates.

    This function retrieves the target coordinates from the ROS parameter server and 
    returns them as a response to the service request.

    :param request: The service request. It is not used in this case, but is required by the service signature.
    :type request: lastTargetRequest
    :returns: The last target coordinates (X and Y).
    :rtype: lastTargetResponse
    """
    posX = float(rospy.get_param("/des_pos_x"))
    posY = float(rospy.get_param("/des_pos_y"))
    return lastTargetResponse(posX, posY)

def main():
    rospy.init_node("targetServiceNode")

    # Register the service with the name '/target_service'
    rospy.Service("/target_service", lastTarget, handleTargetRequest)
    rospy.loginfo("Target service is ready to provide the last target coordinates.")

    # Keep the node running
    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr("Service node interrupted")
        sys.exit(1)

if __name__ == "__main__":
    main()
