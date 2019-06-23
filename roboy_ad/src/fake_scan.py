#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple IMU signal conversion node taking input from sbg_driver
## custom message type and publishing linear acceleration to 
## ROS /imu topic in standard IMU message format

import rospy
from sensor_msgs.msg import LaserScan

class Converter:
	def __init__(self):

		# set up ROS node
		rospy.init_node('scan_converter', anonymous=False)
		self.pub = rospy.Publisher('fake_scan', LaserScan, queue_size=10)
		rospy.Subscriber('scan', LaserScan, self.mapping)
		rospy.spin()

	def mapping(self, ScanData):

		# remap data from SBG_IMU_msg to ROS_IMU_msg and publish
		FakeScan = LaserScan()
		# FakeScan = ScanData
		FakeScan.header.frame_id = 'laser'
		FakeScan.header.seq = ScanData.header.seq
		FakeScan.header.stamp = ScanData.header.stamp

		FakeScan.angle_min = ScanData.angle_min
		FakeScan.angle_max = ScanData.angle_max
		FakeScan.angle_increment = ScanData.angle_increment

		FakeScan.time_increment = ScanData.time_increment

		FakeScan.scan_time = ScanData.scan_time

		FakeScan.range_min = ScanData.range_min
		FakeScan.range_max = ScanData.range_max

		i = 0
		FakeScan.ranges = [1] * len(ScanData.ranges)
		FakeScan.intensities = [1] * len(ScanData.intensities)
		for i in range(len(ScanData.ranges)):
			if ScanData.ranges[i] < ScanData.range_min:
			 	FakeScan.ranges[i] = 30.0
				FakeScan.intensities[i] = 430.0
			else:
				FakeScan.ranges[i] = ScanData.ranges[i]
				FakeScan.intensities[i] = ScanData.intensities[i]
		self.pub.publish(FakeScan)

if __name__ == '__main__':
	try:
		Converter()
	except rospy.ROSInterruptException:
		pass




