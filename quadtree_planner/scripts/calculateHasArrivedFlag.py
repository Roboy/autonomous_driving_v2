#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from tf import TransformListener
from std_msgs.msg import Bool

# This script calculates and publishes the hasArrivedFlag
class CalculateHasArrivedFlag:
    def __init__(self):
        self.goal_x = 0
        self.goal_y = 0
        
    def start(self):
        rospy.init_node('calculateHasArrivedFlag')
        self.tf = TransformListener()
        rospy.loginfo('Node calculateHasArrived is started!')
        self.maximum_distance = 1.0
        self.maximum_velocity = 0.0
        self.flag_publisher = rospy.Publisher('/roboy/autonomousdriving/arrival', Bool, queue_size=10)
        rospy.Subscriber('move_base/goal', MoveBaseActionGoal,  self.handle_goal_position, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.handle_ego_velocity, queue_size=1)
        rospy.loginfo('Publishers and subscribers have been created')
        rospy.spin()

    def handle_goal_position(self, goal_pos):
        self.goal_x, self.goal_y = goal_pos.goal.target_pose.pose.position.x, goal_pos.goal.target_pose.pose.position.y
        rospy.loginfo('Goal Position(x=%.2f, y=%.2f) received', self.goal_x, self.goal_y)

    def euclidDistance(self, goal_pos_x, goal_pos_y, current_pos_x, current_pos_y):
	    return np.sqrt((goal_pos_x - current_pos_x)**2 + (goal_pos_y - current_pos_y)**2)

    def handle_ego_velocity(self, cmd_vel):
        self.ego_velocity = np.sqrt(cmd_vel.linear.x**2 + cmd_vel.linear.y**2)
        t = self.tf.getLatestCommonTime("/map", "/base_link")
        position, quaternion = self.tf.lookupTransform("/map", "/base_link", t)         
        current_pos_x, current_pos_y, = position[0], position[1]
        euclid_dist = self.euclidDistance(self.goal_x, self.goal_y, current_pos_x, current_pos_y)
        if( (self.ego_velocity <= self.maximum_velocity) and (euclid_dist <= self.maximum_distance) ):
            self.flag_publisher.publish(True)
            rospy.loginfo('Publishing hasArrived = True')
        else:
            pass  

if __name__ == "__main__":
    CalculateHasArrivedFlag().start()
