"""
Provides access to the latest value of the Rickhaw's steering angle - the angle
between the front wheels and the base.
Note: it's not a separate node, should be started after a wrapping node is
    initialized.
"""
import rospy
from sensor_msgs.msg import JointState

STEERING_JOINT = 'joint_front'


class RickshawSteeringSensor:

    def __init__(self):
        self.latest_sensor_value = 0

    def start(self):
        rospy.Subscriber('/joint_states', JointState,
                         self.update_steering_angle)

    def update_steering_angle(self, joint_states):
        joint_states = zip(joint_states.name,
                           joint_states.position)
        for joint_name, position in joint_states:
            if joint_name == STEERING_JOINT:
                self.latest_sensor_value = position
                return
        rospy.logerr('RickshawSteeringSensor: Could position of the "%s" joint',
                     STEERING_JOINT)

    def get_latest_sensor_value(self):
        return self.latest_sensor_value
