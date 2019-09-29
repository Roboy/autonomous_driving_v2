"""
    Usage:
    target_angle_listener = TargetAngleListener()
    target_angle_listener.start()

    actual_angle_listener = AngleSensorListener()
    actual_angle_listener.start()

    actual_angle = actual_angle_listener.get_latest_actual_angle()
    target_angle = target_angle_listener.get_latest_target_angle()

    Parameters:
    wheel_base - double, distance between the wheels of the model.

"""
import rospy
import numpy as np
from math import atan, pi, floor

from roboy_middleware_msgs.msg import MotorCommand, MotorAngle, MotorConfig
from roboy_middleware_msgs.srv import MotorConfigService
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

MOTOR_CONTROL_POSITION = 0
MOTOR_CONTROL_VELOCITY = 1
MOTOR_CONTROL_DISPLACEMENT = 2

def rad_to_deg(val):
    """
    Converts angle from radian to degree
    :param val: angle in radian
    :type val: float

    :returns: angle in degree (float)
    """
    return val / pi * 180


def deg_to_rad(val):
    """
    Converts angle from degree to radian
    :param val: angle in degree
    :type val: float

    :returns: angle in radian (float)
    """
    return val / 180 * pi

def convert_trans_rot_vel_to_steering_angle(lin_vel, ang_vel, wheelbase):
    """
    Transforms the rotational velocity command from the cmd_vel message and turns it into a steering angle
    Taken from http://docs.ros.org/kinetic/api/teb_local_planner/html/cmd__vel__to__ackermann__drive_8py_source.html

    :param lin_vel: forward velocity of the bike (is set to 1 per default) (m/s)
    :type lin_vel: float
    :param ang_vel: angular velocity of the cmd_vel message
    :type ang_vel: float
    :param wheelbase: distance between front and back axis of the rickshaw (meter)
    :type wheelbase: float

    :returns: steering angle in radian (float)
    """
    if ang_vel == 0 or lin_vel == 0:
        return 0

    radius = lin_vel / ang_vel
    return atan(wheelbase / radius)


class TargetAngleListener:
    """ This Class contains a ROS-node that subscribes to the cmd_vel for the rickshaw. Passes extracted steering angle to the high-level controller.
    Usage:
    This class is used in the steering_controller.py file and acts as an interface between cmd_vel and the high-level controller.
    INFO: This class was mainly written by AD in WiSe18/19 and this documentation was completed by AD2 in SuSe19, therefore documentation may differ from actual functionality.
    
    ...

    Attributes
    ----------
    target_angle : float
        radian value of target steering angle
    wheel_base : float
        distance

    Methods
    ----------
    start()
        starts the subscriber
    listen_to_navigation_controller()
        subscriber for target angle, converts Twist to angle.
    get_latest_target_angle()
        gives back last known target angle (radian).
    """
    def __init__(self):
        self.target_angle = 0

    def start(self):
        """
        Gets ROS-param and start the subscriber.
        :returns: nothing
        """
        self.wheel_base = rospy.get_param('~wheel_base')
        self.listen_to_navigation_controller()

    def listen_to_navigation_controller(self):
        """
        Subscriber for the target angle
        :returns: nothing
        """
        def navigation_commands_receiver(twist):
            """
            Callback function for the subscriber.
            Converts Twist to radian angle
            :param twist: Twist message with the cmd_vel
            :type twist: Twist
            
            :returns: nothing
            """
            angular_velocity = twist.angular.z
            linear_velocity = 1.0
            self.target_angle = convert_trans_rot_vel_to_steering_angle(
                linear_velocity, angular_velocity, self.wheel_base
            )

        rospy.Subscriber('/cmd_vel', Twist, navigation_commands_receiver)

    def get_latest_target_angle(self):
        """
        Getter for target angle
        :returns: last send target angle in radian (float)
        """
        return self.target_angle


class AngleSensorListener:
    """ This Class contains a ROS-node that subscribes to the processed angle value of the rickshaw and smoothes it out. Passes the smooth angle to the high-level controller.
    Usage:
    This class is used in the steering_controller.py file and acts as an interface between unfiltered angle data and the high-level controller.
    INFO: This class was mainly written by AD in WiSe18/19 and this documentation was completed by AD2 in SuSe19, therefore documentation may differ from actual functionality.
    
    ...

    Attributes
    ----------
    actual_angle : float
        radian value of the unfiltered angle data
    smmoth_angle : float
        filtered angle data (radian)
    last_smooth_angle : float
        is only updated if a certain threshold is crossed
    decay : float
        parameter for smoothing
    threshold : float
        parameter for updating last_smooth_angle

    Methods
    ----------
    start()
        starts the subscriber
    listen_to_angle_sensor()
        subcriber for target angle
    get_latest_actual_angle()
        gives back last known steering angle, not processed (radian).
    get_latest_smooth_angle()
        gives back last known steering angle (radian).
    smooth_out()
        reduces the impact of noise on the angle data
    """
    def __init__(self, 
                 decay=0.95, 
                 threshold=0.1 / 180 * pi):
        """
        :param decay: smooth_angle is computed as
                      angle = decay*angle + (1-decay)*new_angle
        :type decay: float
        :param threshold: smooth_angle is updated if difference between the new
                          and the last value is larger than the provided
                          threshold.
        :tpye thrshold: float
        """
        self.actual_angle = 0
        self.smooth_angle = 0
        self.last_smooth_angle = 0
        self.decay = decay
        self.threshold = threshold

    def start(self):
        """
        starts the subscriber
        """
        self.listen_to_angle_sensor()

    def listen_to_angle_sensor(self):
        """
        ROS-subscriber for the unfiltered angle data
        :returns: nothing
        """
        def angle_receiver(angle):
            """
            Callback function for the subcriber, filters the angle data and updates the attributes.
            """
            self.actual_angle = float(angle.data)
            self.smooth_angle = self.smooth_out(float(angle.data))
            if abs(self.smooth_angle - self.last_smooth_angle) > self.threshold:
                self.last_smooth_angle = self.smooth_angle

        rospy.Subscriber('/roboy/middleware/RickshawAngle', Float32,
                         angle_receiver)

    def get_latest_actual_angle(self):
        """
        Last known steering angle, not processed.
        :returns: unfiltered angle (radian, float)
        """
        return self.actual_angle

    def get_latest_smooth_angle(self):
        """
        Last known steering angle.
        Smoothed using the exponential smoothing
        https://en.wikipedia.org/wiki/Exponential_smoothing
        to filter out the noise.
        :returns: last_smooth_angle (radian, float)
        """
        return self.last_smooth_angle

    def smooth_out(self, angle):
        """
        Reduced the impact of noise by filtering the data
        :param angle: current unfiltered angle data (radian)
        :type angle: float
    
        :returns: filtered angle (radian, float)
        """
        return self.decay * self.smooth_angle + (1 - self.decay) * angle


class MyoMuscleController:
    """ This Class contains a ROS-node that configures the myo-muscles and sends commands to them
    Usage:
    This Class is used in the steering_controller.py file and acts as an interface between high-level steering-controller and low-level myo-muscle controller.
    
    ...

    Attributes
    ----------
    fpga_id : int
        id of the fpga the myo-muscle is connected to
    motor_id : int
        id of the slave-select pin the motor is connected to on the FPGA (check wiring).
    init_disp : int
        default displacement values the motor is set to after configuring them

    Methods
    ----------
    start()
        starts the publisher and configures the myo-muscle
    set_control_mode()
       configures the myo-muscle
    send_command()
        sends the displacement command from the controller to the myo-muscle
    """
    def __init__(self, fpga_id, motor_id, init_disp=10):
        """
        :param fpga_id: id of the fpga the myo-muscle is connected to
        :type fpga_id: int
        :param motor_id: id of the slave-select pin the myo-muscle is connected to on the fpga
        :type motor_id: int
        :param init_disp: default displacment values the motor is set to after configuring them
        :type init_disp: int
        """
        self.fpga_id = fpga_id
        self.motor_id = motor_id
        self.init_disp = init_disp

    def start(self):
        """
        starts the publisher for the motor command.
        Call the initial configurment of the myo-muscle
        :returns: nothing
        """
        self.publisher = rospy.Publisher('/roboy/middleware/MotorCommand',
                                         MotorCommand,
                                         queue_size=1)
        rospy.logwarn('CAREFUL! Myo-muscle controller is activated, '
                      'rickshaw will start turning if cmd_vel command is set.')
        self.set_control_mode()

    def set_control_mode(self):
        """
        Calls the ROS-service that configures the myo-muscle
        :returns: nothing
        """
        config_motors_service = rospy.ServiceProxy(
            '/roboy/rikshaw/middleware/MotorConfig',
            MotorConfigService
        )
        config = MotorConfig(
            id=self.fpga_id,
            motors=[self.motor_id],
            control_mode=[MOTOR_CONTROL_DISPLACEMENT],
            output_pos_max=[1000],
            output_neg_max=[-1000],
            sp_pos_max=[1000000],
            sp_neg_max=[-1000000],
            integral_pos_max=[1000],
            integral_neg_max=[-1000],
            kp=[200],
            ki=[0],
            kd=[0],
            forward_gain=[0],
            dead_band=[0],
            output_divider=[1],
            setpoint=[self.init_disp]
        )
        config_motors_service(config)

    def send_command(self, effort):
        """
        Sends displacment commands via ROS-message to the fpga
        :param effort: displacement the myo-muscle should reach
        :type effort: int
        
        :returns: nothing
        """
        command = MotorCommand()
        command.id = self.fpga_id
        command.motors = [self.motor_id]
        command.set_points = [effort]
        self.publisher.publish(command)
