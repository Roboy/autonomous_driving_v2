#!/usr/bin/python

"""
A ROS node that listens initial_position command that can be issued from RVIZ
and updates model's position in gazebo accordingly
"""

import rospy

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseHandler:

    def __init__(self):
        pass

    def start(self):
        rospy.init_node('handle_initial_pos', log_level=rospy.DEBUG)

        self.get_params()
        self.set_robot_pos = rospy.Publisher('/gazebo/set_model_state',
                                        ModelState, queue_size=1)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped,
                         self.handle_initial_position, queue_size=1)
        rospy.spin()

    def get_params(self):
        self.name = rospy.get_name()
        self.model_name = rospy.get_param('%s/model_name' % self.name)

    def handle_initial_position(self, pose):
        rospy.loginfo('%s: Resetting poisition of the %s',
                      self.name, self.model_name)
        pos = pose.pose.pose
        robot_state = ModelState()
        robot_state.model_name = self.model_name
        robot_state.pose = pos
        self.set_robot_pos.publish(robot_state)


if __name__ == '__main__':
    InitialPoseHandler().start()