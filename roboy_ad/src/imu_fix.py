#!/usr/bin/env python

import rosbag as rb
import rospy as rp
import sys
import time
import os
import argparse
from sensor_msgs.msg import Imu


def status(length, percent):
    """
    writes a status bar to the system's stdout

    Input:
    - length (int): length of the status bar (number of characters
    - percent (float): progress percentage that is reached
    """
    sys.stdout.write('\x1B[2K')  # Erase entire current line
    sys.stdout.write('\x1B[0E')  # Move to the beginning of the current line
    visual_progress = int(length * percent)
    progress = "Progress: [" \
               + visual_progress * '=' + int(length - visual_progress) * ' ' \
               + "] " + str(round(percent * 100.0, 2)) + "%"
    sys.stdout.write(progress)
    sys.stdout.flush()


def fix_z_msg(imu_msg):

    fixed_msg = Imu()
    fixed_msg.header = imu_msg.header

    fixed_msg.linear_acceleration.x = imu_msg.linear_acceleration.x
    fixed_msg.linear_acceleration.y = imu_msg.linear_acceleration.y
    fixed_msg.linear_acceleration.z = 9.8073

    fixed_msg.angular_velocity = imu_msg.angular_velocity
    fixed_msg.orientation_covariance = imu_msg.orientation_covariance

    return fixed_msg


def fix_z_imu(bagfile):

    bag = rb.Bag(bagfile)

    topics = bag.get_type_and_topic_info()[1].keys()
    start_time = bag.get_start_time()
    duration = bag.get_end_time() - start_time

    root, ext = os.path.splitext(bagfile)
    newfile = root + '_fix-z' + ext
    newbag = rb.Bag(newfile, 'w')

    if not '/imu' in topics:
        print('no imu data to fix')
        print('topics in current bag file: {}'.format(topics))
        return


    last_time = time.clock()

    for topic, msg, t in bag.read_messages():

        # update status bar every 100 ms
        if time.clock() - last_time > .1:
            percent = (t.to_sec() - start_time) / duration
            status(40, percent)
            last_time = time.clock()

        if topic == '/imu':
            fixed_msg = fix_z_msg(msg)
            newbag.write(topic, fixed_msg, t)
        else:
            newbag.write(topic, msg, t)

    status(40, 1)
    print('\nfinished successfully')

if __name__ == '__main__':

    parser = argparse.ArgumentParser('fix z direction of linear acceleration of imu messages to gravitation')
    parser.add_argument('bag_file', help='path to the bagfile whose messages should be edited')

    args = parser.parse_args(rp.myargv()[1:])

    fix_z_imu(args.bag_file)