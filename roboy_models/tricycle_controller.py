#!/usr/bin/python
"""
A controller that listens to cmd_vel topics and forwards commands to the
tricycle model running in gazebo. Model's definition can be found in
roboy_models/tricycle/model.urdf
"""
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Twist
from math import pi

EPS = 1e-04
WHEEL_RADIUS = 0.1

class TricycleControlGazebo:

    def __init__(self):
        pass


    def start(self):
        rospy.init_node('tricycle_control_gazebo', log_level=rospy.DEBUG)
        wheels_pub = rospy.Publisher(
            '/tricycle_wheels_controller/command', Float64MultiArray,
            queue_size=10)
        steering_pub = rospy.Publisher(
            '/tricycle_steering_controller/command', Float64,
            queue_size=10)

        def handle_velocity(twist):
            vx, vy = twist.linear.x, twist.linear.y
            if vy > EPS:
                rospy.logerr('Non-holomonic velocity found vy=%.2f', vy)
            phi = twist.angular.z
            rospy.logdebug('Command(vel=%.2f, steering=%.2f) received', vx, phi)
            steering_pub.publish(phi)
            wheel_p = 2.0 * WHEEL_RADIUS * pi
            wheel_rot_vel = vx / wheel_p
            lin_vel = Float64MultiArray(data=[wheel_rot_vel, wheel_rot_vel,
                                              wheel_rot_vel])

            wheels_pub.publish(lin_vel)

        rospy.Subscriber('cmd_vel', Twist, handle_velocity, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    controller = TricycleControlGazebo()
    controller.start()
