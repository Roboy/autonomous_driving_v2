#!/usr/bin/env python

import rospy
import time
import argparse
import yaml
import os
import rospkg
from math import atan, pi, tan
from roboy_middleware_msgs.msg import MotorCommand, MotorAngle, MotorConfig
from roboy_middleware_msgs.srv import MotorConfigService
from geometry_msgs.msg import Twist


def convert_trans_rot_vel_to_steering_angle(lin_vel, ang_vel, wheelbase):
    # gives back steering_angle in rad
    if ang_vel == 0 or lin_vel == 0:
        return 0

    radius = lin_vel / ang_vel
    return atan(wheelbase / radius)

def convert_steering_angle_to_trans_rot_vel(lin_vel, angle, wheelbase):
    ang_vel = lin_vel*tan(angle)/wheelbase
    return ang_vel

class StartUpSequence:

    def __init__(self, sequence, zero_angle_raw=2600, 
                 right_angle_raw=2570, right_angle=30,
                 left_angle_raw=1810, left_angle=-30, 
                 decay=0.95, threshold=0.1/180 * pi, threshold_angle=2):
        self.rate = 5
        self.sequence = sequence
        print(self.sequence)
        print('sequence lenght: {}'.format(len(self.sequence)))
        # angles
        self.target_angle = 0
        self.actual_angle = 0
        self.smooth_angle = 0
        self.last_smooth_angle = 0
        self.decay = decay
        self.threshold = threshold
        self.zero_angle_raw = zero_angle_raw
        self.right_angle_raw = right_angle_raw
        self.right_angle = right_angle
        self.left_angle_raw = left_angle_raw
        self.left_angle = left_angle
        self.threshold_angle = threshold_angle
        self.send_true = True
        #timing
        self.timeout_start = time.time()
        self.timeout_time = 10
        self.hold_start = time.time()
        self.timing_started = False
        self.hold_time = 10
        self.counter = 0
        
    def start(self):
        self.wheel_base = 1.6
        #self.wheel_base = rospy.get_param('~wheel_base')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('seq_tx', anonymous=True)
        self.listen_to_angle_sensor()
        self.send_msg()

    def send_msg(self):
        vel_msg = Twist()
        lin_vel = 1.0
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.counter < len(self.sequence):
                vel_msg.linear.x = convert_steering_angle_to_trans_rot_vel(lin_vel, 
                                                                           self.sequence[self.counter]['linear']*pi/180, 
                                                                           self.wheel_base)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = convert_steering_angle_to_trans_rot_vel(lin_vel, 
                                                                            self.sequence[self.counter]['angular']*pi/180, 
                                                                            self.wheel_base)
                self.pub.publish(vel_msg)
                if not self.send_true:
                    self.send_true = True
                    self.counter += 1
                    if self.counter >= len(self.sequence):
                        rospy.signal_shutdown('Finished Startup')
                self.target_angle = self.sequence[self.counter]['angular']*pi/180
                self.timeout_time = self.sequence[self.counter]['timeout']
                self.hold_time = self.sequence[self.counter]['holdtime']
            rate.sleep()  


    def listen_to_angle_sensor(self):
        def angle_receiver(raw_angle):
            # get angles
            if len(raw_angle.raw_angles) != 1:
                rospy.logerr('Invalid motor_angle command received')
            angle = float(raw_angle.raw_angles[0] - self.zero_angle_raw) \
                    / (self.right_angle_raw - self.left_angle_raw) \
                    * (self.right_angle - self.left_angle) * pi / 180
            self.actual_angle = angle
            self.smooth_angle = self.smooth_out(angle)
            if abs(self.smooth_angle - self.last_smooth_angle) > self.threshold:
                self.last_smooth_angle = self.smooth_angle

            # check if timeout has occured
            if (not self.timing_started) and ((time.time() - self.timeout_start) > self.timeout_time):
                rospy.signal_shutdown('Failed Startup')
                        
            # check if actual angle stays in reach of desired angle for a certain time
            if abs(self.target_angle - self.last_smooth_angle) < (self.threshold_angle * pi/180) and self.send_true:
                # start timer if threshold is frist crossed
                if not self.timing_started:
                    print('start timing')
                    self.timing_started = True
                    self.hold_start = time.time()
                # check if time threshhold has been crossed
                elif (time.time() - self.hold_start) > self.hold_time:
                    print('sending_next message: {}'.format(self.counter + 1))
                    self.send_true = False
                    self.timeout_start = time.time()
            # reset timer if angle leaves threshold again 
            elif self.timing_started:
                self.timing_started = False
         
        rospy.Subscriber('/roboy/middleware/StearingAngle', MotorAngle,
                         angle_receiver)

    def smooth_out(self, angle):
        return self.decay * self.smooth_angle + (1 - self.decay) * angle


if __name__ == '__main__':
    config_path = os.path.join(rospkg.RosPack().get_path('roboy_navigation'), 'config')
    # calibration
    with open(os.path.join(config_path, 'calibration.yaml'), 'r') as ymlfile:
        calibration = yaml.load(ymlfile)
    # startup sequence
    with open(os.path.join(config_path, 'startup.yaml'), 'r') as ymlfile:
        sequence = yaml.load(ymlfile)

    StartUpSequence(sequence, calibration['raw']['raw_middle'], calibration['raw']['raw_right'], calibration['angles']['right'], 
        calibration['raw']['raw_left'], calibration['angles']['left']).start()
