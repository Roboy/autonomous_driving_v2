#!/usr/bin/env python
import sys
import rospy
import argparse
import time
from roboy_cognition_msgs.srv import DriveToLocation

"""
.. module:: luigi_test_client
    :platform: ROS
    :synopsis:  script is supposed to be used only as test script for the communication between 
    Luigi and autononomous driving via the service DriveToLocation (located in roboy_cognition_msgs)

.. moduleauthor:: Maximilian Kempa <maximilian.kempa@tum.de>
"""


def ad_communication(location):
    """Method to simulate a location request from luigi

    Args:
        location (string): string that describes the location where Luigi wants the rickshaw drive to
    """
    rospy.wait_for_service('autonomous_driving')
    try:
        drive_to_location = rospy.ServiceProxy('autonomous_driving', DriveToLocation)
        response = drive_to_location(location)
        return response.eta, response.error_message, response.path_found
    except rospy.ROSInterruptException as e:
        print('Service call failed:', e)


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Define goal location from datatype string")
    parser.add_argument('--destination', '-d', default = 'midnightsurprise', help ='string that is sent as destination to the service DriveToLocation')
    args = parser.parse_args(rospy.myargv()[1:])
    eta, error_message, path_found = ad_communication(args.destination)
    print("eta: ", eta, " error_message: ", error_message, " path_found: ", path_found)
    time.sleep(5)
