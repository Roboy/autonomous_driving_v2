#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple IMU signal conversion node taking input from sbg_driver
## custom message type and publishing linear acceleration to 
## ROS /imu topic in standard IMU message format

import rospy
from sensor_msgs.msg import Imu
from sbg_driver.msg import SbgImuData

class Converter:
	def __init__(self):

		# set up ROS node
		rospy.init_node('imu_data_converter', anonymous=False)
		self.pub = rospy.Publisher('imu', Imu, queue_size=10)
		rospy.Subscriber('imu_data', SbgImuData, self.mapping)
		rospy.spin()

	def mapping(self, SBG_IMU_msg):

		# remap data from SBG_IMU_msg to ROS_IMU_msg and publish
		ROS_IMU_msg = Imu()
		ROS_IMU_msg.header.frame_id = 'imu'
		ROS_IMU_msg.header.seq = SBG_IMU_msg.header.seq
		ROS_IMU_msg.header.stamp = SBG_IMU_msg.header.stamp

		ROS_IMU_msg.linear_acceleration.x = SBG_IMU_msg.accel.x
		ROS_IMU_msg.linear_acceleration.y = SBG_IMU_msg.accel.y
		ROS_IMU_msg.linear_acceleration.z = SBG_IMU_msg.accel.z
		ROS_IMU_msg.angular_velocity.x = SBG_IMU_msg.gyro.x
		ROS_IMU_msg.angular_velocity.y = SBG_IMU_msg.gyro.y
		ROS_IMU_msg.angular_velocity.z = SBG_IMU_msg.gyro.z

		ROS_IMU_msg.orientation_covariance[0] = -1

		self.pub.publish(ROS_IMU_msg)

if __name__ == '__main__':
	try:
		Converter()
	except rospy.ROSInterruptException:
		pass




