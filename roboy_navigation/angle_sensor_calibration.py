#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import os
import yaml
from roboy_middleware_msgs.msg import MotorAngle

class AngleSensorCalibration:

    def __init__(self, num_msgs=100):
        self.num_msgs = num_msgs
        self.rate = 1
        self.target_angles = ('middle', 'right', 'left')
        self.default_angles = dict(middle = 0,
                                   right = -30,
                                   left = 30)
        self.pkg_path = rospkg.RosPack().get_path('roboy_navigation')
        self.angles = {}
        self.raw = {}

    def start(self):
        

        rospy.init_node('angle_calibration', anonymous=True)
        rate = rospy.Rate(self.rate)
        for angle in self.target_angles:
            print('Move the rickshaw to the {}'.format(angle))
            try:
                input_angle = int(input('Which angle is this? \n Please specify the angle. If you want to take the maximum angle of {} press enter '.format(self.default_angles[angle])))
            except SyntaxError or ValueError:
                print('Going on with default value.')
                input_angle = self.default_angles[angle]
            try:
                input('If rickshaw is in position press Enter...')
            except SyntaxError:
                pass
            print('Hold in place')

            self.angles[angle] = input_angle

            self.data = []
            self.counter = 0
            for self.counter in range(self.num_msgs):
                message = rospy.wait_for_message('/roboy/middleware/StearingAngle', MotorAngle)
                self.data.append(message.raw_angles[0])
                rate.sleep()
            collected_data = np.asarray(self.data)
            raw_angle = 'raw_' + angle
            self.raw[raw_angle] = int(collected_data.mean())
            print('You can release the rickshaw.')
        calibration = dict(raw = self.raw,
                           angles = self.angles)

        with open(os.path.join(self.pkg_path, 'config/calibration.yaml'), 'w') as ymlfile:
            yaml.dump(calibration, ymlfile, default_flow_style=False)


if __name__=='__main__':
    AngleSensorCalibration(num_msgs=10).start()
  
