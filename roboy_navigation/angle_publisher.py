#!/usr/bin/python
import argparse
import rospy

from std_msgs.msg import Float64

from roboy_navigation.steering_helper import TargetAngleListener, \
    AngleSensorListener, rad_to_deg


class AnglePublisher:

    def __init__(self, zero_angle_raw=2640, rate=20):
        self.rate = rate
        self.angle_sensor_listener = AngleSensorListener(zero_angle_raw)
        self.target_angle_listener = TargetAngleListener()

    def start(self):
        rospy.init_node('steering_angle_publisher')
        #TODO: refactor
        rospy.set_param('~wheel_base', 1.6)
        self.angle_sensor_listener.start()
        self.actual_angle_publisher = rospy.Publisher('/actual_angle', Float64,
                                                      queue_size=10)
        self.smooth_angle_publisher = rospy.Publisher('smooth_angle', Float64,
                                                      queue_size=10)
        self.target_angle_listener.start()
        self.target_angle_publisher = rospy.Publisher('/target_angle', Float64,
                                                      queue_size=10)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            actual_angle = self.angle_sensor_listener.get_latest_actual_angle()
            self.actual_angle_publisher.publish(rad_to_deg(actual_angle))
            smooth_angle = self.angle_sensor_listener.get_latest_smooth_angle()
            self.smooth_angle_publisher.publish(rad_to_deg(smooth_angle))
            target_angle = self.target_angle_listener.get_latest_target_angle()
            self.target_angle_publisher.publish(rad_to_deg(target_angle))
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ROS Node to publish steering '
                                                 'and commanded angles.')
    parser.add_argument('--zero_angle_raw', type=int, default=2640)
    parser.add_argument('--sample_rate', type=int, default=20)
    args, _ = parser.parse_known_args()
    print('angle_publisher config:')
    print(args)
    AnglePublisher(args.zero_angle_raw, args.sample_rate).start()