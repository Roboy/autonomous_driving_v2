#!/usr/bin/env python

import rospy
import time
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

    def __init__(self, sequence, zero_angle_raw=2600, decay=0.95, threshold=0.1/180 * pi, threshold_angle=2):
        self.counter = 0
        self.rate = 5
        self.sequence = sequence
        self.target_angle = 0
        self.actual_angle = 0
        self.smooth_angle = 0
        self.last_smooth_angle = 0
        self.decay = decay
        self.threshold = threshold
        self.zero_angle_raw = zero_angle_raw
        self.threshold_angle = threshold_angle
        self.send_true = True
        
    def start(self):
        self.wheel_base = 1.6
        #self.wheel_base = rospy.get_param('~wheel_base')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('seq_tx', anonymous=True)
        self.listen_to_angle_sensor()
        self.send_msg()

    def shutdown_msg():
        print('shutting down node: sending commands')

    def send_msg(self):
        vel_msg = Twist()
        lin_vel = 1.0
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.counter < len(sequence):
                vel_msg.linear.x = convert_steering_angle_to_trans_rot_vel(lin_vel, sequence[self.counter]['linear']*pi/180, self.wheel_base)
                vel_msg.linear.y = 0 #sequence[self.counter]['linear']['y']
                vel_msg.linear.z = 0 #sequence[self.counter]['linear']['z']
                vel_msg.angular.x = 0 #sequence[self.counter]['angular']['x']
                vel_msg.angular.y = 0 #sequence[self.counter]['angular']['y']
                vel_msg.angular.z = convert_steering_angle_to_trans_rot_vel(lin_vel, sequence[self.counter]['angular']*pi/180, self.wheel_base)
                self.pub.publish(vel_msg)
                if not self.send_true:
                    self.send_true = True
                    self.counter += 1
                self.target_angle = sequence[self.counter]['angular']*pi/180
                #print(self.target_angle)#convert_trans_rot_vel_to_steering_angle(
                    #lin_vel, sequence[self.counter]['angular'], self.wheel_base)
                
            else:
                rospy.signal_shutdown('Finished Startup')
            rate.sleep()

   
    def listen_to_angle_sensor(self):
        def angle_receiver(raw_angle):
            if len(raw_angle.raw_angles) != 1:
                rospy.logerr('Invalid motor_angle command received')
            angle = float(
                raw_angle.raw_angles[0] - self.zero_angle_raw) \
                                / 4096 * 2 * pi
            self.actual_angle = angle
            self.smooth_angle = self.smooth_out(angle)
            if abs(self.smooth_angle - self.last_smooth_angle) > self.threshold:
                self.last_smooth_angle = self.smooth_angle
            #self.last_smooth_angle = angle

            if abs(self.target_angle - self.last_smooth_angle) < (self.threshold_angle * pi/180) and self.send_true:
                print('sending_next message: ', self.counter)
                self.send_true = False
         
        rospy.Subscriber('/roboy/middleware/StearingAngle', MotorAngle,
                         angle_receiver)
        #rospy.spin()

    def smooth_out(self, angle):
        return self.decay * self.smooth_angle + (1 - self.decay) * angle


if __name__ == '__main__':
    sequence = [{'linear': 0.0, 'angular': 10},
                {'linear': 0.0, 'angular': -10},
                {'linear': 0.0, 'angular': 20},
                {'linear': 0.0, 'angular': -20},
                {'linear': 0.0, 'angular': 0}]
    StartUpSequence(sequence).start()