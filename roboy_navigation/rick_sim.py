#!/usr/bin/env python

import rospy
import time
from math import atan, pi, exp, floor
from roboy_middleware_msgs.msg import MotorCommand, MotorAngle, MotorConfig
from geometry_msgs.msg import Twist


def convert_trans_rot_vel_to_steering_angle(lin_vel, ang_vel, wheelbase):
    # gives back steering_angle in rad
    if ang_vel == 0 or lin_vel == 0:
        return 0

    radius = lin_vel / ang_vel
    return atan(wheelbase / radius)

def convert_angle_to_encoder(angle, zero_angle_raw=2600):
    encoder = floor(angle / (2*pi) * 4096 + zero_angle_raw)
    return encoder

class LowPassSim:

    def __init__(self):
        self.counter = 0
        self.starting_angle = 0
        self.goal_angle = 0
        self.angle = MotorAngle()
        self.rate = 5
        self.time_constant = 0.1
        self.current_angle = 0
        self.wheel_base = 1.6
        
    def start(self):
        self.pub = rospy.Publisher('/roboy/middleware/StearingAngle', MotorAngle, queue_size=1)
        self.angle.id = 0
        self.angle.raw_angles = [0]
        self.angle.raw_angles_prev = [0]
        self.angle.offset_angles = [0]
        self.angle.relative_angles = [0]
        self.angle.rev_counter = [0]
        self.counter = 0

        self.rick_node()

    def rick_node(self):
        rospy.init_node('send_angle', anonymous=True)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.listen_to_target_angle()
            self.current_angle = self.goal_angle - ((self.goal_angle - self.starting_angle) * exp(- self.time_constant * self.counter/self.rate))
            #self.angle.raw_angles = [2600]
            self.angle.raw_angles = [convert_angle_to_encoder(self.current_angle)]
            self.pub.publish(self.angle)
            self.counter += 1
            rate.sleep()

    def shutdown_msg():
        print('shutting down node: sending angle')
   
    def listen_to_target_angle(self):
        def navigation_commands_receiver(twist):
            angular_velocity = twist.angular.z
            linear_velocity = 1.0
            self.new_goal_angle = convert_trans_rot_vel_to_steering_angle(
                linear_velocity, angular_velocity, self.wheel_base
            )
            if self.goal_angle != self.new_goal_angle:
                self.counter = 0
                self.goal_angle = self.new_goal_angle
                self.starting_angle = self.current_angle
                print('angles: ', self.goal_angle, self.starting_angle)
            
        rospy.Subscriber('/cmd_vel', Twist, navigation_commands_receiver)


if __name__ == '__main__':
    LowPassSim().start()
