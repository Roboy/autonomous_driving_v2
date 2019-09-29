#!/usr/bin/env python

import rospy
import yaml
import os
import rospkg
from math import pi
from roboy_middleware_msgs.msg import MotorAngle
from std_msgs.msg import Float32

class AngleSensorListener:
    """
    Class for a ROS-node, that subscribes to the angle sensor of the Rickshaw and evalutaes its data. Publishes the processed data as the acutal value in radian.
    Usage:
    This ROS-node should be either used via the startup.launch file or the control_start.launch file.

    ...

    Attributes
    ----------
    zero_angle_raw : int
        raw measurement value of the angle sensor when the rickshaw was at 0 degrees (stored in config/calibration.yaml)
    right_angle_raw : int
        raw measurement value of the angle sensor when the rickshaw was turned to the right (stored in config/calibration.yaml)
    left_angle_raw : int
        raw measurement value of the angle sensor when the rickshaw was turned to the left (stored in config/calibration.yaml)
    right_angle : int
        angle measurment when the rickshaw tunred to the right (stored in config/calibration.yaml)
    left_angle : int
        angle measurment when the rickshaw tunred to the left (stored in config/calibration.yaml)

    Methods
    ----------
    start()
        starts the node (subscriber and publisher) 
    listen_to_angle_sensor()
        starts the subscriber and processes the raw angle data to the acutal angle in radian
    """    
    def __init__(self, zero_angle_raw=2190, 
                 right_angle_raw=2570, right_angle=30,
                 left_angle_raw=1810, left_angle=-30):
        """
        :param zero_angle_raw: sensor value corresponding to zero steering angle.
        :type zero_angle_raw: int
        :param right_angle_raw: sensor value corresponding to right_angle
        :type right_angle_raw: int
        :param left_angle_raw: sensor value corresponding to left_angle
        :type left_angle_raw: int
        :param right_angle: angle, to which the rickshaw was turned to the right
        :type right_angle: int
        :param left_angle: angle, to which the rickshaw was turned to the left
        :type left_angle: int
        """
        self.zero_angle_raw = zero_angle_raw
        self.right_angle_raw = right_angle_raw
        self.right_angle = right_angle
        self.left_angle_raw = left_angle_raw
        self.left_angle = left_angle

    def start(self):
        """
        starts the ROS-node (publisher and subscriber)
        :returns: nothing
        """
        self.pub = rospy.Publisher('/roboy/middleware/RickshawAngle', Float32, queue_size=1)
        rospy.init_node('true_angle_publisher', anonymous=True)
        self.listen_to_angle_sensor()

    def listen_to_angle_sensor(self):
        """
        Starts the subscriber to the angle sensor
        :returns: nothing
        """
        def angle_receiver(raw_angle):
            """
            Callback-function for the subscriber.
            Processes the data from raw values to actual angle. Publsihes the actual angle (radian)
            Raw sensor data is stored in the raw_angles of the message
            :param raw_angle: raw sensor information
            
            :returns: nothing
            """
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
