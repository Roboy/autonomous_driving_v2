#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import os

class AngleSensorCalibration:

    def __init__(self, num_msgs=10):
        self.num_msgs = num_msgs
        self.counter = 0
        self.target_angles = ('middle', 'right', 'left')
        self.default_angles = dict(middle = 0,
                                   right = -30,
                                   left = 30)
        self.pkg_path = rospkg.RosPack().get_path('roboy_navigation')
        self.angles = {}
        self.raw = {}

    def start(self):
        rospy.init_node('angle_calibration', anonymous=True)
        for angle in self.target_angles:
            print('Move the rickshaw to the {}'.format(angle))
            try:
                input_angle = input('Which angle is this? \n Please specify the angle. If you want to take the maximum angle of {} press enter'.format(self.default_angles[angle]))
            except SyntaxError:
                input_angle = self.default_angles[angle]
            if type(angle) != int:
                input_angle = self.default_angles[angle]
            try:
                input('If rickshaw is in position press Enter...')
            except SyntaxError:
                pass

            self.angles[angle] = input_angle

            data = ()
            for i in range(num_msgs):
                data = rospy.wait_for_message('/roboy/middleware/StearingAngle', MotorAngle)
                data.append(data.raw_angles[0])
            collected_data = np.asarray(data)
            raw_angle = 'raw' + angle
            self.raw[raw_angle] = int(collected_data.mean())
        calibration = dict(raw = self.raw,
                           angles = self.angles)

        with open(os.path.join(self.pkg_path, 'config/calibration.yaml'), 'w') as ymlfile:
        yaml.dump(calibration, ymlfile, default_flow_style=False)

        output = dict()
        f.open("raw_values", "w")
        f.write(str(self.raw))
        f.close()
  
