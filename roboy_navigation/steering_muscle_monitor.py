#!/usr/bin/env python
'''
This node allows easy monitoring of steering muscle monitors. 
It prints warnings whenever there are no 'good' messages. 
A good message means that the steering muscles are working correctly. 
This means that at least 2 currents in the corresponding MotorStatus message are not zero. 
'''


import rospy
import time
from roboy_middleware_msgs.msg import MotorStatus
import datetime


ID = 5
N_CURRENTS_OVER_TRESHOLD = 2
CURRENT_THRESHOLD = 0
TIME_BETWEEN_2_GOOD_MESSAGES = 1.0

ENABLE_ONE_MESSAGE_WARN = False # print warning for every single 'bad' message
ENABLE_TIME_DELTA_WARN = True # print warning only if there was no good message in time delta TIME_BETWEEN_2_GOOD_MESSAGES
ENABLE_GOOD_MESSAGE = False # print warning for every single 'good' message


class MuscleMonitor:
    def __init__(self):
        rospy.Subscriber("/roboy/middleware/MotorStatus", MotorStatus, self.callback)
        self.first_message_received = False
        self.time_of_last_good_message = None

    def callback(self, msg):
        if not self.first_message_received:
            print("First MotorStatus message received. Start monitoring...\n")
            self.first_message_received = True
        self.check_health(msg)


    def check_health(self, msg):
        '''
                uint8 id -> weniger als 2 currents ueber 0 -> log warning; ->
                bool power_sense
                int32[] pwm_ref
                int32[] position
                int32[] velocity
                int32[] displacement
                int16[] current
                int32[] angle
                '''
        if msg.id == ID:
            n_over_current = 0
            for current in msg.current:
                if current > CURRENT_THRESHOLD:
                    n_over_current += 1
            if n_over_current < N_CURRENTS_OVER_TRESHOLD:
                if ENABLE_ONE_MESSAGE_WARN:
                    rospy.logwarn("MotorStatus with ID %i had less than 2 currents > 0" % ID)
            else:
                self.time_of_last_good_message = datetime.datetime.now()
                if ENABLE_GOOD_MESSAGE:
                    rospy.loginfo("Good message received")


    def run(self):
        rospy.init_node('steering_muscle_monitor', anonymous=True)
        try:
            rate = rospy.Rate(2.0)
            while not rospy.is_shutdown():
                if self.first_message_received:
                    break
                print(".")
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        #regularly check
        try:
            rate = rospy.Rate(10.0)
            while not rospy.is_shutdown():
                if self.time_of_last_good_message is not None:
                    timedelta = datetime.datetime.now() - self.time_of_last_good_message
                    if timedelta.total_seconds() > TIME_BETWEEN_2_GOOD_MESSAGES:
                        if ENABLE_TIME_DELTA_WARN:
                            rospy.logwarn('%f seconds have passed since last good message' % timedelta.total_seconds())
                rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    monitor = MuscleMonitor()
    print("Started script. Waiting for first MotorStatus message to be received...")
    monitor.run()

