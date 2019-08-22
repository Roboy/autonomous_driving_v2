#!/usr/bin/env python

import rospy
import time
import yaml
import os
import rospkg
from math import atan, pi, tan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def convert_steering_angle_to_trans_rot_vel(lin_vel, angle, wheelbase):
    ang_vel = lin_vel*tan(angle)/wheelbase
    return ang_vel

class StartUpSequence:

    def __init__(self, sequence, decay=0.95, 
                 threshold=0.1/180 * pi, threshold_angle=2):
        self.rate = 5
        self.sequence = sequence
        
        # angles
        self.target_angle = 0
        self.actual_angle = 0
        self.smooth_angle = 0
        self.last_smooth_angle = 0
        self.decay = decay
        self.threshold = threshold
        self.send_next = False
        #timing
        self.timeout_start = time.time()
        self.hold_start = time.time()
        self.timing_started = False
        self.threshold_angle = threshold_angle
        self.target_angle = self.sequence[0]['angular']*pi/180
        self.timeout_time = self.sequence[0]['timeout']
        self.hold_time = self.sequence[0]['holdtime']
        self.counter = 0
        
    def start(self):
        print('Starting a sequence of lenght: {}'.format(len(self.sequence)))
        self.wheel_base = 1.6
        #self.wheel_base = rospy.get_param('~wheel_base')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('seq_tx', anonymous=True)
        self.listen_to_angle_sensor()
        print('Sending first message.')
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
                if self.send_next:
                    self.send_next = False
                    self.counter += 1
                    if self.counter >= len(self.sequence):
                        self.default_position()
                        self.pub.publish(vel_msg)
                        print('Finished startup sequence.')
                        rospy.signal_shutdown('Finished Startup')
                    else:
                        print('Sending next message: {}'.format(self.counter + 1))
                        # reset values
                        self.target_angle = self.sequence[self.counter]['angular']*pi/180
                        self.timeout_time = self.sequence[self.counter]['timeout']
                        self.hold_time = self.sequence[self.counter]['holdtime']
                        self.timeout_start = time.time()
            rate.sleep()  


    def listen_to_angle_sensor(self):
        def angle_receiver(angle):
            self.actual_angle = float(angle.data)
            self.smooth_angle = self.smooth_out(float(angle.data))
            if abs(self.smooth_angle - self.last_smooth_angle) > self.threshold:
                self.last_smooth_angle = self.smooth_angle

            # check if timeout has occured
            if (not self.timing_started) and ((time.time() - self.timeout_start) > self.timeout_time):
                vel_msg = self.default_position()
                self.pub.publish(vel_msg)
                print("Couldn't finish startup sequence please check the hardware. Going to default position.")
                rospy.signal_shutdown('Failed Startup')
                        
            # check if actual angle stays in reach of desired angle for a certain time
            if abs(self.target_angle - self.last_smooth_angle) < (self.threshold_angle * pi/180) and not self.send_next:
                # start timer if threshold is frist crossed
                if not self.timing_started:
                    print('Reached goal position, start timing')
                    self.timing_started = True
                    self.hold_start = time.time()
                # check if time threshhold has been crossed
                elif (time.time() - self.hold_start) > self.hold_time:
                    print('Finished message')
                    self.send_next = True
                    self.timing_started = False
            # reset timer if angle leaves threshold again 
            elif self.timing_started:
                print('Stopped timing.')
                self.timing_started = False
         
        rospy.Subscriber('/roboy/middleware/TrueAngle', Float32,
                         angle_receiver)


    def smooth_out(self, angle):
        return self.decay * self.smooth_angle + (1 - self.decay) * angle

    def default_position(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        return vel_msg


if __name__ == '__main__':
    config_path = os.path.join(rospkg.RosPack().get_path('roboy_navigation'), 'config')
    # calibration
    with open(os.path.join(config_path, 'calibration.yaml'), 'r') as ymlfile:
        calibration = yaml.load(ymlfile)
    # startup sequence
    with open(os.path.join(config_path, 'startup.yaml'), 'r') as ymlfile:
        sequence = yaml.load(ymlfile)

    StartUpSequence(sequence).start()
