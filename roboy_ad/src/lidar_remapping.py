#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple Lidar signal conversion node taking input from livox_ros_driver
## takes header timestamp and changes it to current ROS time

import rospy
import genpy
from sensor_msgs.msg import PointCloud2

class Converter:
        def __init__(self):

                # set up ROS node
                rospy.init_node('livox_lidar_converter', anonymous=False)
                self.pub = rospy.Publisher('points2', PointCloud2, queue_size=10)
                rospy.Subscriber('livox/lidar', PointCloud2, self.mapping)
                rospy.spin()

        def mapping(self, livox_msg):

                # remap data from SBG_IMU_msg to ROS_IMU_msg and publish
                points_msg = PointCloud2()
                points_msg = livox_msg
                #print("Old timestamp: {}".format(points_msg.header.stamp))
                points_msg.header.stamp = genpy.rostime.Time(secs = rospy.Time.now().to_sec())
                #print("New timestamp: {}".format(points_msg.header.stamp))
                #print("Difference to current time in [s]: {}".format(points_msg.header.stamp.to_sec() - genpy.rostime.Time(secs = rospy.Time.now().to_sec()).to_sec()))

                self.pub.publish(points_msg)

if __name__ == '__main__':
        try:
                Converter()
        except rospy.ROSInterruptException:
                pass

