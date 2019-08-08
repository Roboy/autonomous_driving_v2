#!/usr/bin/env python

import rospy
import numpy as np
import argparse

class AngleSensorCalibration:

    def __init__(self, num_msgs=10):
        self.num_msgs = num_msgs
        self.counter = 0
        self.target_angles = ('middle', 'right', 'left')
        self.data = ()
        self.average = ()
        self.raw = {}

    def start(self):
        for angle in self.target_angles:
            print('Move the rickshaw to the {}'.format(angle))
            input('If rickshaw is in position press Enter...')
            for i in range(num_msgs):
                data = rospy.wait_for_message('/roboy/middleware/StearingAngle', MotorAngle)
                self.data.append(data.raw_angles[0])
            collected_data = np.asarray(self.data)
            self.average.append(int(collected_data.mean()))
        
        self.raw['middle'] = self.average[0]
        self.raw['right'] = self.average[1]
        self.raw['left'] = self.average[2]
        f.open("raw_values", "w")
        f.write(str(self.raw))
        f.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ROS node to calibrate the angle sensor')
    parser.add_argument('--num_msgs', type=int, required=True)
    args, _ = parser.parse_known_args()
    print('Calibration:')
    AngleSensorCalibration(args.num_msgs).start()
  
