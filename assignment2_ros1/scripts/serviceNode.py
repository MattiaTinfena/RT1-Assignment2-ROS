#!/usr/bin/env python3

import rospy
import sys
from assignment2_ros1.srv import lastTarget, lastTargetResponse  # Import custom service message

# Callback function for the service
def handleTargetRequest(request):
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
