#!/usr/bin/env python
import sys
import rospy
import argparse
from roboy_cognition_msgs.srv import DriveToLocation

# This script is supposed to be used only as test script for the communication between Luigi and autononomous driving via the service DriveToLocation (located in roboy_cognition_msgs)
def ad_communication(location):
    rospy.wait_for_service('autonomous_driving')
    try:
        drive_to_location = rospy.ServiceProxy('autonomous_driving', DriveToLocation)
        response = drive_to_location(location)
        return response.eta, response.error_message
    except rospy.ROSInterruptException as e:
        print('Service call failed:', e)
# If driving module is run without ROS, comment everything from above (including imports) and uncomment this:
    # return 42, ""

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Define goal location from datatype string")
    parser.add_argument('--destination', '-d', default = 'midnightsurprise', help ='string that is sent as destination to the service DriveToLocation')
    args = parser.parse_args(rospy.myargv()[1:])
    ad_communication(args.destination)
