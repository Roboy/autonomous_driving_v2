#!/usr/bin/python
"""
A controller that listens to cmd_vel topics and forwards commands to the
tricycle model running in gazebo. Model's definition can be found in
simulation/tricycle/model.urdf
"""
import rospy

from math import pi, atan
from simple_pid import PID
from threading import Thread

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

from angle_sensor import SteeringAngleSensor

EPS = 1e-04
WHEEL_RADIUS = 0.25
WHEEL_BASE = 1.6


def convert_trans_rot_vel_to_steering_angle(lin_vel, ang_vel, wheelbase):
    if ang_vel == 0 or lin_vel == 0:
        return 0

    radius = lin_vel / ang_vel
    return atan(wheelbase / radius)


class SteeringController:

    def __init__(self, sample_rate=20, Kp=10, Ki=0.1, Kd=0.05):
        self.sample_rate = sample_rate
        self.pid = PID(Kp, Ki, Kd, setpoint=0)
        self.steering_sensor = SteeringAngleSensor()
        self.steering_pub = rospy.Publisher(
            '/tricycle_front_part_controller/command', Float64,
            queue_size=10)
        self.target_angle = 0

    def start(self):
        self.steering_sensor.start()
        Thread(target=self._run).run()

    def _run(self):
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            actual_angle = self.steering_sensor.get_latest_sensor_value()
            error = actual_angle - self.target_angle
            steering_vel = self.pid(error)
            self.steering_pub.publish(steering_vel)
            rate.sleep()


class TricycleControlGazebo:

    def __init__(self):
        self.steering_controller = SteeringController()

    def start(self):
        rospy.init_node('tricycle_control_gazebo', log_level=rospy.DEBUG)
        self.wheels_pub = rospy.Publisher(
            '/tricycle_wheels_controller/command', Float64MultiArray,
            queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.handle_velocity_command,
                         queue_size=1)
        self.steering_controller.start()
        rospy.spin()

    def handle_velocity_command(self, twist):
        target_lin_vel, vy = twist.linear.x, twist.linear.y
        if vy > EPS:
            rospy.logerr('Non-holonomic velocity found vy=%.2f', vy)
        target_rot_vel = twist.angular.z
        rospy.logdebug('Command(lin_vel=%.2f, rot_vel=%.2f) received',
                       target_lin_vel, target_rot_vel)
        self.set_steering_vel(target_lin_vel, target_rot_vel)
        self.set_wheels_vel(target_lin_vel)

    def set_steering_vel(self, target_lin_vel, target_rot_vel):
        self.steering_controller.target_angle = convert_trans_rot_vel_to_steering_angle(
            target_lin_vel, target_rot_vel, WHEEL_BASE)

    def set_wheels_vel(self, target_lin_vel):
        wheel_p = 2.0 * WHEEL_RADIUS * pi
        wheel_rot_vel = target_lin_vel / wheel_p
        rospy.logdebug('Try setting wheels speed to %.2f', wheel_rot_vel)
        wheels_vel = Float64MultiArray(data=[wheel_rot_vel, wheel_rot_vel,
                                             wheel_rot_vel])
        self.wheels_pub.publish(wheels_vel)


if __name__ == '__main__':
    controller = TricycleControlGazebo()
    controller.start()
