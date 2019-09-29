#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import os
import yaml
from roboy_middleware_msgs.msg import MotorAngle

class AngleSensorCalibration:
    """ Class to calibrate an angle sensor and store the calibration it a yaml config file.
    Usage:
    Its intended use is for the angle sensor on the rickshaw.
    It guides the user through the calibration process. The dafault process consist of three steps. The user has to manually move the rickshaw into three positions (he can use the default angle value or define new ones) and then this ROS-node will listen to a given number of messages by the angle sensor and average over them. This process includes three steps to ensure, that the middle position of the rickshaw has the correct angle. The other two point determine the resolution of the angle sensor.
    This code can be modified for a different calibration process, furthermore sanitiy should be included. E.g. is the middle angle approx. in the middle of the other two angles.

    ...

    Attributes
    ----------
    num_msgs : int
        number of messages the class listens to and averages over
    rate : int
        freuquency of how often the listener checks for new messages
    target_angles : list of str
        points defined by the user, to which to steer the rickshaw to
    default_angles: dict of int
        degree values of the points defined by the user
    pkg_path: str
        path to the roboy_navigation package, needed for writing data into config file
    angles: dict of int
        actual chosen angle values by the user
    raw: dict of int
        average of the respective sensor angle values
        

    Methods
    ----------
    start()
        starts the calibration process
    """

    def __init__(self, num_msgs=100):
        """
        :param num_msgs: number of messages to average over (default: 100)
        :type num_msgs: int
        """
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
        """
        Starts the calibration process. Listens to messages of the angle sensor.
        Writes the collected data into a yaml file in the config directory.
        :returns: nothing
        """
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
                message = rospy.wait_for_message('/roboy/middleware/SteeringAngle', MotorAngle)
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
  
