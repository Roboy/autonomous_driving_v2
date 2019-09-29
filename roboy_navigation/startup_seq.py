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
    """ Helper function that converts the desired steering angle into a correspinding lin_vel used by the cmd_vel message using the geometry of the rickshaw.
    :param lin_vel: the forward velocity of the rickshaw in m/s
    :type lin_vel: float
    :param angle: the desired angle the rickshaw should move to
    :type angle: float
    :param wheelbase: the distance between the front wheels of the rickshaw and the back wheel
    :type wheelbase: float

    :returns: angular velocity used by the cmd_vel message
    :type return: float
    """
    ang_vel = lin_vel*tan(angle)/wheelbase
    return ang_vel

class StartUpSequence:
    """ This class performes a predefined startup sequence to ensure the steering and driving motor is working as intended. This class gets a startup sequence defined at config/startup.yaml and sends the corresponding commands to the controller ROS-node via the cmd_vel msg. It then listens to the steering angle sensor to check if the goal position was reached. If the position was reached and hold for a few seconds it will proceed with the next entry in the startup sequence. If a goal position is not reached within a certain time limit this node will terminate, tell the user it couldn't finish and exit.
    Usage:
    This ROS-node should be used via the startup launch file, which also launches the controller scripts and the angle-converter.
    
    ...

    Attributes
    ----------
    rate : int
        freuquency of how often the publisher publishes new messages
    sequence :  dictionary
        startup sequence that is executed defined in config/startup.yaml
    target_angle : float
        the current target angle that the rickshaw should steer to (radian)
    actual angle : float
        the current angle, the rickshaw is at the moment (radian)
    smooth_angle : float
        processed rickshaw-angle to smooth noise (radian)
    last_smooth_angle : float
        updates only if  the smooth_angle changes by a threshold, so the angle is less volatile
    decay : float
        parameter to smooth the rickshaw angle
    threshold : float
        parameter for last_smooth_angle
    send_next : bool
        Flag to indicate, if the next command should be send
    timeout_start : float
        timestamp, when the current goal was first published
    timeout_time : float
        time in seconds in which the goal position has to be reached, otherwise startup-sequence has failed
    hold_start : float
        timestamp, when the current goal-zone was initaily reached
    hold_time : float
        time in seconds how long the angle has to stay inside the goal-zone to fullfil the current command
    timing_started : bool
        Flag to indicate if the timing in the current goal position has already started
        

    Methods
    ----------
    start()
        starts the startup sequence process
    send_msg()
        publishes the current goal position to the control-node
    listen_to_angle_sensor()
        listens to the rickshaw angle, starts timing, terminates ROS-node if timeout has occured
    smooth_out()
        processing of angle sensor data
    default_position()
        returns a default rickshaw configuration, no speed and 0 degree angle TODO: define as Attribute    
    """
    def __init__(self, sequence, decay=0.95, 
                 threshold=0.1/180 * pi, threshold_angle=2):
        """
        :param sequence: startup-sequence given by config-file
        :type sequnce: dictionary
        :param decay: parameter for angle smoothing (default 0.95)
        :type decay: float
        :param threshold: parameter for last_smooth_angle; determines, when last_smooth_angle is updated (radian, default 0.1/180 * pi)
        :type threshold: float
        :param threshold_angle: tolerance for goal-range (degree, default 2)
        :type threshold_angle: int
        """
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
        """
        Start the startup sequence.
        Initializes the ROS-node and the publisher. Calls and initializes the subscriber and then starts sending cmd_vel messages
        """
        print('Starting a sequence of lenght: {}'.format(len(self.sequence)))
        self.wheel_base = 1.6
        #self.wheel_base = rospy.get_param('~wheel_base')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.init_node('seq_tx', anonymous=True)
        self.listen_to_angle_sensor()
        print('Sending first message.')
        print('Angle: {}, Velocity: {}'.format(self.target_angle, self.sequence[self.counter]['linear']))
        self.send_msg()

    def send_msg(self):
        """
        Constanly publishes cmd_vel messages for the controller to receive
        Gets the message values from the sequence dictionary and processes them to a ROS Twist message.
        Iterates through the sequence if a goal is reached and sets the new parameters
        """
        vel_msg = Twist()
        lin_vel = 1.0
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.counter < len(self.sequence):
                vel_msg.linear.x = self.sequence[self.counter]['linear']
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
                        vel_msg = self.default_position()
                        self.pub.publish(vel_msg)
                        rospy.sleep(2)
                        print('Finished startup sequence.')
                        rospy.signal_shutdown('Finished Startup')
                    else:
                        print('Sending next message: {}'.format(self.counter + 1))
                        print('Angle: {}, Velocity: {}'.format(self.target_angle, self.sequence[self.counter]['linear']))
                        # reset values
                        self.target_angle = self.sequence[self.counter]['angular']*pi/180
                        self.timeout_time = self.sequence[self.counter]['timeout']
                        self.hold_time = self.sequence[self.counter]['holdtime']
                        self.timeout_start = time.time()
            rate.sleep()  


    def listen_to_angle_sensor(self):
        """
        Starts subscriber for RickshawAngle message
        Listens to the radian angle of the rickshaw_angle sensor and call the callback function.
        """
        def angle_receiver(angle):
            """
            Callback function for the subscriber
            Smoothes the angle data and checks for timeout or if a goal is reached.
            Mannges timing.
            """
            self.actual_angle = float(angle.data)
            self.smooth_angle = self.smooth_out(float(angle.data))
            if abs(self.smooth_angle - self.last_smooth_angle) > self.threshold:
                self.last_smooth_angle = self.smooth_angle

            # check if timeout has occured
            if (not self.timing_started) and ((time.time() - self.timeout_start) > self.timeout_time):
                vel_msg = self.default_position()
                self.pub.publish(vel_msg)
                rospy.sleep(2)
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
         
        rospy.Subscriber('/roboy/middleware/RickshawAngle', Float32,
                         angle_receiver)


    def smooth_out(self, angle):
        """
        Function to reduce the effect of noise on the angle sensor, get a smoother reading.
        :param angle: current reading of the angle sensor (radian)
        :type angle: float
        """
        return self.decay * self.smooth_angle + (1 - self.decay) * angle

    def default_position(self):
        """
        Defines a default position for the Rickshaw to revert back to, once the startup sequence has finished or failed.
        TODO: better as an attribute
        """
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
