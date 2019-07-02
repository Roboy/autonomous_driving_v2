#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


EPS = 1e-04


class RickshawController:
    """
    A ROS node that receives cmd_vel commands and converts them to commands for rickshaw.

    Binary controller. When given a non-zero target speed, it will turn on the motor.
    Otherwise
    """
    def __init__(self):
        pass

    def start(self):
        rospy.init_node('rickshaw_controller', log_level=rospy.DEBUG)
        rospy.logwarn('CAREFUL! Rickshaw motor controller is activated, '
                      'rickshaw will start driving if cmd_vel command is set.')

        self.motor_publisher = rospy.Publisher('/roboy/control/GPIO', Bool,
                                               queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.handle_velocity_command,
                         queue_size=1)
        rospy.spin()

    def handle_velocity_command(self, twist):
        vx, vy = twist.linear.x, twist.linear.y
        if vy > EPS:
            rospy.logerr('Non-holomonic velocity found vy=%.2f', vy)
        phi = twist.angular.z
        rospy.logdebug('Command(vel=%.2f, steering=%.2f) received', vx, phi)
        self.update_speed(vx)

    def update_speed(self, target_speed):
        if target_speed > EPS:
            self.motor_publisher.publish(True)
        else:
            self.motor_publisher.publish(False)


if __name__ == '__main__':
    RickshawController().start()