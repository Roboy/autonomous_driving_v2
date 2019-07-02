#!/usr/bin/python
"""
A node that will republish tf transform information to the odometry topic.
Can be used instead of doing actual odometry but it will not provide model
velocity.
"""
import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

DEFAULT_WORLD_FRAME = '/map'
DEFAULT_MODEL_FRAME = 'base_link'


class TFToOdom:

    def __init__(self):
        pass

    def start(self):
        rospy.init_node('tf_to_odom',
                        log_level=rospy.DEBUG)
        self.get_params()
        self.forward_odom()

    def get_params(self):
        self.name = rospy.get_name()
        self.model_frame = rospy.get_param('%s/model_frame' % self.name,
                                           default=DEFAULT_MODEL_FRAME)
        self.world_frame = rospy.get_param('%s/world_frame' % self.name,
                                           default=DEFAULT_WORLD_FRAME)

    def forward_odom(self):
        listener = tf.TransformListener()
        odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform(
                    self.world_frame, self.model_frame, rospy.Time(0))
                # publish to 'odom' topic
                curr_time = rospy.Time.now()
                odom_msg = Odometry()
                odom_msg.header.stamp = curr_time
                odom_msg.header.frame_id = self.world_frame
                odom_msg.child_frame_id = self.model_frame
                odom_msg.pose.pose.position = Point(trans[0], trans[1],
                                                    trans[2])
                odom_msg.pose.pose.orientation = Quaternion(rot[0], rot[1],
                                                            rot[2], rot[3])
                odom_pub.publish(odom_msg)
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue


if __name__ == '__main__':
    TFToOdom().start()
