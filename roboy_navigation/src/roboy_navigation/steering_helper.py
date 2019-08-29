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
    return val / pi * 180


def deg_to_rad(val):
    return val / 180 * pi


# Taken from http://docs.ros.org/kinetic/api/teb_local_planner/html/cmd__vel__to__ackermann__drive_8py_source.html
def convert_trans_rot_vel_to_steering_angle(lin_vel, ang_vel, wheelbase):
    if ang_vel == 0 or lin_vel == 0:
        return 0

    radius = lin_vel / ang_vel
    return atan(wheelbase / radius)


class TargetAngleListener:

    def __init__(self):
        self.target_angle = 0

    def start(self):
        self.wheel_base = rospy.get_param('~wheel_base')
        self.listen_to_navigation_controller()

    def listen_to_navigation_controller(self):
        def navigation_commands_receiver(twist):
            angular_velocity = twist.angular.z
            linear_velocity = 1.0
            self.target_angle = convert_trans_rot_vel_to_steering_angle(
                linear_velocity, angular_velocity, self.wheel_base
            )

        rospy.Subscriber('/cmd_vel', Twist, navigation_commands_receiver)

    def get_latest_target_angle(self):
        return self.target_angle


class AngleSensorListener:

    def __init__(self, 
                 decay=0.95, 
                 threshold=0.1 / 180 * pi):
        """
        :param decay: smooth_angle is computed as
                      angle = decay*angle + (1-decay)*new_angle
        :param threshold: smooth_angle is updated if difference between the new
                          and the last value is larger than the provided
                          threshold.

        """
        self.actual_angle = 0
        self.smooth_angle = 0
        self.last_smooth_angle = 0
        self.decay = decay
        self.threshold = threshold

    def start(self):
        self.listen_to_angle_sensor()

    def listen_to_angle_sensor(self):
        def angle_receiver(angle):
            self.actual_angle = float(angle.data)
            self.smooth_angle = self.smooth_out(float(angle.data))
            if abs(self.smooth_angle - self.last_smooth_angle) > self.threshold:
                self.last_smooth_angle = self.smooth_angle

        rospy.Subscriber('/roboy/middleware/RickshawAngle', Float32,
                         angle_receiver)

    def get_latest_actual_angle(self):
        """
        Last known steering angle, not processed.
        """
        return self.actual_angle

    def get_latest_smooth_angle(self):
        """
        Last known steering angle.
        Smoothed using the exponential smoothing
        https://en.wikipedia.org/wiki/Exponential_smoothing
        to filter out the noise.
        """
        return self.last_smooth_angle

    def smooth_out(self, angle):
        return self.decay * self.smooth_angle + (1 - self.decay) * angle


class MyoMuscleController:

    def __init__(self, fpga_id, motor_id, init_disp=10):
        self.fpga_id = fpga_id
        self.motor_id = motor_id
        self.init_disp = init_disp

    def start(self):
        self.publisher = rospy.Publisher('/roboy/middleware/MotorCommand',
                                         MotorCommand,
                                         queue_size=1)
        rospy.logwarn('CAREFUL! Myo-muscle controller is activated, '
                      'rickshaw will start turning if cmd_vel command is set.')
        self.set_control_mode()

    def set_control_mode(self):
        config_motors_service = rospy.ServiceProxy(
            '/roboy/spine_right/middleware/MotorConfig',
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
        command = MotorCommand()
        command.id = self.fpga_id
        command.motors = [self.motor_id]
        command.set_points = [effort]
        self.publisher.publish(command)
