#!/usr/bin/env python

import rospy
import yaml
import os
import rospkg
from math import pi
from roboy_middleware_msgs.msg import MotorAngle
from std_msgs.msg import Float32

class AngleSensorListener:

    def __init__(self, 
                 zero_angle_raw=2190, 
                 right_angle_raw=2570, right_angle=30,
                 left_angle_raw=1810, left_angle=-30, 
                 decay=0.95, threshold=0.1 / 180 * pi):
        """
        :param zero_angle_raw: sensor value corresponding to zero steering angle.
        :param decay: smooth_angle is computed as
                      angle = decay*angle + (1-decay)*new_angle
        :param threshold: smooth_angle is updated if difference between the new
                          and the last value is larger than the provided
                          threshold.

        """
        self.zero_angle_raw = zero_angle_raw
        self.right_angle_raw = right_angle_raw
        self.right_angle = right_angle
        self.left_angle_raw = left_angle_raw
        self.left_angle = left_angle

    def start(self):
        self.pub = rospy.Publisher('/roboy/middleware/TrueAngle', Float32, queue_size=1)
        rospy.init_node('true_angle_publisher', anonymous=True)
        self.listen_to_angle_sensor()

    def listen_to_angle_sensor(self):
        def angle_receiver(raw_angle):
            if len(raw_angle.raw_angles) != 1:
                rospy.logerr('Invalid motor_angle command received')
            angle = float(raw_angle.raw_angles[0] - self.zero_angle_raw) \
                    / (self.right_angle_raw - self.left_angle_raw) \
                    * (self.right_angle - self.left_angle) * pi / 180
            self.pub.publish(angle)

        rospy.Subscriber('/roboy/middleware/SteeringAngle', MotorAngle,
                         angle_receiver)
        rospy.spin()

if __name__ == '__main__':
    config_path = os.path.join(rospkg.RosPack().get_path('roboy_navigation'), 'config')
    # calibration
    with open(os.path.join(config_path, 'calibration.yaml'), 'r') as ymlfile:
        calibration = yaml.load(ymlfile)

    AngleSensorListener(calibration['raw']['raw_middle'], calibration['raw']['raw_right'], calibration['angles']['right'], 
        calibration['raw']['raw_left'], calibration['angles']['left']).start()
