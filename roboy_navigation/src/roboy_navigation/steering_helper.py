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
from math import atan, pi

from roboy_middleware_msgs.msg import MotorCommand, MotorAngle, MotorConfig
from roboy_middleware_msgs.srv import MotorConfigService
from geometry_msgs.msg import Twist

MOTOR_CONTROL_POSITION = 0
MOTOR_CONTROL_VELOCITY = 1
MOTOR_CONTROL_DISPLACEMENT = 2

RICKSHAW_MAX_ANGLE = 33
MOTOR_POS_X = 60
MOTOR_POS_Y = 25
MOTOR_POS_Z = 10
LIFTING_ARM_POS_X = 65
LIFTING_ARM_POS_Y = 12
LIFTING_ARM_POS_Z = 0

def get_compensation():
    compensation = {}

    motor_r_static = np.array([MOTOR_POS_X, MOTOR_POS_Y, MOTOR_POS_Z])
    motor_l_static = np.array([MOTOR_POS_X, -MOTOR_POS_Y, MOTOR_POS_Z])
    liftingarm_r = np.array([LIFTING_ARM_POS_X, -LIFTING_ARM_POS_Y, LIFTING_ARM_POS_Z])
    liftingarm_l = np.array([LIFTING_ARM_POS_X, LIFTING_ARM_POS_Y, LIFTING_ARM_POS_Z])
    max_angle = RICKSHAW_MAX_ANGLE

    for angle in range(-max_angle, max_angle, 1):
        angle_rad = (angle + 0.5) * np.pi /180
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        R = np.array(((c,-s,0),(s,c,0),(0,0,1)))
        # calc motor rotation postion
        mot_r = np.matmul(R, motor_r_static)
        mot_l = np.matmul(R, motor_l_static)

        tendon_r = mot_r - liftingarm_r
        tendon_l = mot_l - liftingarm_l
        cross_r = np.cross(liftingarm_r, tendon_r)
        cross_l = np.cross(liftingarm_l, tendon_l)

        compensation[angle] = (abs(np.divide(np.linalg.norm(tendon_r), cross_r[2])),
	                       abs(np.divide(np.linalg.norm(tendon_l), cross_l[2])))
    return compensation

def testing_seq():
    sequence = [(0, 30),(0, -30),(0, 0),(0, 15),(0, -15)]
    return sequence

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

    def __init__(self, zero_angle_raw=2450, decay=0.95, threshold=0.1 / 180 * pi):
        """
        :param zero_angle_raw: sensor value corresponding to zero steering angle.
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
        self.zero_angle_raw = zero_angle_raw

    def start(self):
        self.listen_to_angle_sensor()

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

        rospy.Subscriber('/roboy/middleware/StearingAngle', MotorAngle,
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

    def __init__(self, fpga_id, left_motor_id, right_motor_id, init_disp=10):
        self.fpga_id = fpga_id
        self.left_motor_id = left_motor_id
        self.right_motor_id = right_motor_id
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
            '/roboy/unknown/middleware/MotorConfig',
            MotorConfigService
        )
        config = MotorConfig(
            id=self.fpga_id,
            motors=[self.left_motor_id, self.right_motor_id],
            control_mode=[MOTOR_CONTROL_DISPLACEMENT, MOTOR_CONTROL_DISPLACEMENT],
            output_pos_max=[1000, 1000],
            output_neg_max=[-1000, -1000],
            sp_pos_max=[1000000, 1000000],
            sp_neg_max=[-1000000, -1000000],
            integral_pos_max=[1000, 1000],
            integral_neg_max=[-1000, -1000],
            kp=[200, 200],
            ki=[0, 0],
            kd=[0, 0],
            forward_gain=[0, 0],
            dead_band=[0, 0],
            output_divider=[1, 1],
            setpoint=[self.init_disp, self.init_disp]
        )
        config_motors_service(config)

    def send_command_right(self, effort_right):
        command = MotorCommand()
        command.id = self.fpga_id
        command.motors = [self.right_motor_id]
        command.set_points = [effort_right]
        self.publisher.publish(command)

    def send_command_left(self, effort_left):
        command = MotorCommand()
        command.id = self.fpga_id
        command.motors = [self.left_motor_id]
        command.set_points = [effort_left]
        self.publisher.publish(command)
