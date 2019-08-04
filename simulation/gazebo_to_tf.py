#!/usr/bin/python
"""
A node that publishes model position from gazebo to tf. Model's name in gazebo
and model from in tf are allowed to differ.
"""

import rospy
import tf

from gazebo_msgs.msg import ModelStates

DEFAULT_MODEL_NAME = 'tricycle'
DEFAULT_WORLD_FRAME = '/map'
DEFAULT_MODEL_FRAME = 'base_link'

START_DELAY = 3  # sec


class GazeboToTFNode:

    def __init__(self):
        pass

    def start(self):
        rospy.init_node('gazebo_to_tf', log_level=rospy.DEBUG)
        self.get_params()
        rospy.sleep(START_DELAY)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/gazebo/model_states/', ModelStates,
                         self.publish_model_state, queue_size=10)
        rospy.spin()

    def get_params(self):
        self.name = rospy.get_name()
        self.model_name = rospy.get_param('%s/model_name' % self.name)
        self.model_frame = rospy.get_param('%s/model_frame' % self.name,
                                           default=DEFAULT_MODEL_FRAME)
        self.world_frame = rospy.get_param('%s/world_frame' % self.name,
                                           default=DEFAULT_WORLD_FRAME)

    def publish_model_state(self, model_states):
        model_states = zip(model_states.name,
                           model_states.pose)
        for model_name, pose in model_states:
            if model_name == self.model_name:
                self.broadcast_tf(pose)
                return
        rospy.logerr('%s: Could not find position and velocity of %s',
                     self.name, self.model_name)

    def broadcast_tf(self, position):
        curr_time = rospy.Time.now()
        # Publish to tf
        lin_pos = position.position
        quat = position.orientation
        self.tf_broadcaster.sendTransform(
            translation=(lin_pos.x, lin_pos.y, lin_pos.z),
            rotation=(quat.x, quat.y, quat.z, quat.w),
            time=curr_time,
            child=self.model_frame,
            parent=self.world_frame)


if __name__ == '__main__':
    GazeboToTFNode().start()
