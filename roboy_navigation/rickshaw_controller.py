#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


EPS = 1e-04


class RickshawController:
    """
    A ROS-node that receives cmd_vel commands and converts them to commands for main driving motor of the rickshaw.
    Binary controller. When given a non-zero target speed, it will turn on the motor, else stop it.
    Usage:
    This node is used by the startup.launch and the controller_start.launch files and will activate the motors if a cmd_vel is set.

    ...

    Attributes
    ----------

    Methods
    ----------
    start()
        starts the node, the publsiher for motor_commands and the subcriber.
    handle_velocity_command()
        callback for the subscriber, reads the cmd_vel and gives back infos
    update_speed()
        turns on/off the motor, when given a target speed
    """
    def __init__(self):
        """
        nothing
        """
        pass

    def start(self):
        """
        Starts the ROS-node, the publisher, that sends commands to the FPGA, as well as the subscriber for the cmd_vel
        :returns: nothing
        """
        rospy.init_node('rickshaw_controller', log_level=rospy.DEBUG)
        rospy.logwarn('CAREFUL! Rickshaw motor controller is activated, '
                      'rickshaw will start driving if cmd_vel command is set.')

        self.motor_publisher = rospy.Publisher('/roboy/control/GPIO', Bool,
                                               queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.handle_velocity_command,
                         queue_size=1)
        rospy.spin()

    def handle_velocity_command(self, twist):
        """
        Callback function for the subscriber
        Analyses the cmd_vel, and checks for errors, if non found, updates the speed command for the FPGA, according to the linear vel. in x direction.
        :param twist: cmd_vel from the navigation ROS-node
        :type twist: Twist
    
        :returns: nothing
        """
        vx, vy = twist.linear.x, twist.linear.y
        if vy > EPS:
            rospy.logerr('Non-holomonic velocity found vy=%.2f', vy)
        phi = twist.angular.z
        rospy.logdebug('Command(vel=%.2f, steering=%.2f) received', vx, phi)
        self.update_speed(vx)

    def update_speed(self, target_speed):
        """
        Updates the speed command for the FPGA, by sending a ROS-message
        :param target_speed: Wanted linear speed in x-direction
        :type target_speed: float

        :returns: nothing
        """
        if target_speed > EPS:
            self.motor_publisher.publish(True)
        else:
            self.motor_publisher.publish(False)


if __name__ == '__main__':
    RickshawController().start()
