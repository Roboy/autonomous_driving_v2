#!/usr/bin/python
"""
A controller that listens to cmd_vel topics and forwards commands to the
rickshaw model running in gazebo. Model's definition can be found in
roboy_models/rickshaw/model.urdf
"""
import rospy

from math import pi, atan


from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

from rickshaw_angle_sensor import RickshawSteeringSensor
from roboy_navigation.async_pid import AsyncPID


EPS = 1e-04
WHEEL_RADIUS = 0.25
WHEEL_BASE = 1.6


def convert_trans_rot_vel_to_steering_angle(lin_vel, ang_vel, wheelbase):
    if ang_vel == 0 or lin_vel == 0:
        return 0

    radius = lin_vel / ang_vel
    return atan(wheelbase / radius)


class RickshawControlGazebo:

    def __init__(self):
        self.target_steering_angle = 0
        target_steering_anble_fn = lambda: self.target_steering_angle
        self.steering_sensor = RickshawSteeringSensor()
        self.steering_controller = AsyncPID(
            target_steering_anble_fn,
            self.steering_sensor.get_latest_sensor_value,
            self.set_steering_velocity,
            Kp=1.0, Kd=0.0, Ki=0.0,
            sample_rate = 20
        )

    def start(self):
        rospy.init_node('rickshaw_control_gazebo', log_level=rospy.DEBUG)
        self.steering_sensor.start()
        self.wheels_pub = rospy.Publisher(
            '/rickshaw_wheels_controller/command', Float64MultiArray,
            queue_size=10)
        self.steering_pub = rospy.Publisher(
            '/rickshaw_front_part_controller/command', Float64,
            queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.handle_velocity_command, queue_size=1)
        self.steering_controller.start()
        rospy.spin()

    def handle_velocity_command(self, twist):
        target_lin_vel, vy = twist.linear.x, twist.linear.y
        if vy > EPS:
            rospy.logerr('Non-holomonic velocity found vy=%.2f', vy)
        target_rot_vel = twist.angular.z
        self.target_steering_angle = convert_trans_rot_vel_to_steering_angle(
            target_lin_vel, target_rot_vel, WHEEL_BASE)
        rospy.logdebug('Command(lin_vel=%.2f, rot_vel=%.2f) received',
                       target_lin_vel, target_rot_vel)
        wheel_p = 2.0 * WHEEL_RADIUS * pi
        wheel_rot_vel = target_lin_vel / wheel_p
        rospy.logdebug('Try setting wheels speed to %.2f', wheel_rot_vel)
        wheels_vel = Float64MultiArray(data=[wheel_rot_vel, wheel_rot_vel,
                                             wheel_rot_vel])
        self.wheels_pub.publish(wheels_vel)

    def set_steering_velocity(self, steering_vel):
        self.steering_pub.publish(steering_vel)


if __name__ == '__main__':
    controller = RickshawControlGazebo()
    controller.start()
