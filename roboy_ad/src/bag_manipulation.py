#!/usr/bin/env python

import rosbag as rb
import rospy as rp
import rosnode as rn
import genpy as gp
import numpy as np
import sys
import os
import time
import argparse
import click
import struct
from sensor_msgs.msg import PointCloud2


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

def convert_to_float(msg):
    data = np.zeros((len(msg.data) / 4,1))
    for i in range(len(msg.data) / 4):
        raw_data = msg.data[4*i:4*(i+1)]
        data[i] = struct.unpack('<f', raw_data)
    data = np.reshape(data, (-1, 4))
    return data


class BagOrder:

    def __init__(self, bagfile, outpath):
        """
        Input:
        - bagfile (str): path to a ROS bag file (*.bag)
        - outpath (str): path to directory where reordered bag should be saved (if None, save in root directory of bagfile)
        """

        if os.path.isfile(bagfile) and bagfile.lower().endswith('.bag'):  # check for correct type of file
            self.file = bagfile
            self.bag = rb.Bag(bagfile)
            self.topics = self.bag.get_type_and_topic_info()[1].keys()  # store topics contained in bag
            self.types = []
            for i in range(0, len(self.bag.get_type_and_topic_info()[1].values())):
                self.types.append(self.bag.get_type_and_topic_info()[1].values()[i][0])
                # store msg types contained in bag

            if outpath and os.path.isdir(outpath):
                filename, ext = os.path.splitext(os.path.split(self.file)[1])
                self.sorted_file = outpath + filename + '_sort' + ext
                self.sorted_bag = rb.Bag(self.sorted_file, 'w')
            else:
                root, ext = os.path.splitext(self.file)  # make path and bag for sorted messages
                self.sorted_file = root + '_sort' + ext
                self.sorted_bag = rb.Bag(self.sorted_file, 'w')

        else:
            raise IOError("Argument path does not point to a valid bag file.\n"
                          + "Make sure, that a valid path to a bag file (*.bag) is passed to this class.")

    def set_param(self, reference_topic, axis, offset, mirror):
        if type(axis) != list:
            self.axis = [axis]
        else:
            self.axis = axis
        if type(offset) != list:
           self.offset = [offset]
        else:
           self.offset = offset
        if len(self.axis) != len(self.offset):
           raise IOError("Arguments axis (-a) and offset (-off) do not have the same length. \n"
                         + "Make sure to pass lists of the same length")
        self.ref_topic = reference_topic
        self.mirror = mirror
            

    def add_offset(self):
#        self.ordered_data[:,axis:(axis+1)] += offset
        for axis, offset in zip(self.axis, self.offset):
            #print(type(self.ordered_data[:,axis:(axis+1)]))
            self.ordered_data[:,axis:(axis+1)] += offset

    def mirror_data(self):
	for axis in self.axis:
            self.ordered_data[:,axis:(axis+1)] = -self.ordered_data[:,axis:(axis+1)]

    def write_bag_file(self, msg):
        self.ordered_data = self.ordered_data.reshape(-1,1)
	msg_string = ''
        for i in range(self.ordered_data.size):
            msg_string += struct.pack('f', self.ordered_data[i])
	msg.data = msg_string
        self.new_bag.write(self.ref_topic, msg)


    def read_array_data(self, reference_topic, output_bag, num_columns=4):
        """
        Adds an offset to a axis of the LIDAR data
        Writes sorted messages into new bagfile
        Output:
        - bagfile_offset (str): path to file (*.bag) in which messages are ordered chronologically
        """

        print('Adding offset...')

        if reference_topic not in self.topics:
            print('The selected reference topic is not one of the topics contained in the bag file.')
            print('You can see which topics are present in the bag file, calling \"rosbag info <path/to/bagfile>\" '
                  'from your terminal')
            return -1
	self.new_bag = rb.Bag(output_bag, 'w')
        counter = 0.0
        total = self.bag.get_message_count(reference_topic)
        print(total)
        for topic, msg, t in self.bag.read_messages(topics=[reference_topic]):
            self.ordered_data = convert_to_float(msg)
            self.add_offset()
            #if self.mirror:
            #    self.mirror_data()
            self.write_bag_file(msg)
            if counter % 100 ==0:
                percent = counter/total
                status(40, percent)
            counter += 1
        self.new_bag.close



if __name__ == '__main__':

    parser = argparse.ArgumentParser('Reorder an existing bag file')
    parser.add_argument('bag_file', help='path to the bagfile whose messages should be reordered')
    parser.add_argument('--ref_topic', '-rf', help='topic name which acts as a reference for the time stamps')
    parser.add_argument('--axis', '-a', nargs='+', help='choose which axis to add an offset to', type=int)
    parser.add_argument('--offset', '-off', nargs='+', help='define the size of the offset', type=int)
    parser.add_argument('--mirror', '-m', help='mirror the axis after adding the offset', type=bool)
    parser.add_argument('--out_path', '-o', help='path to file with reordered bag file')

    args = parser.parse_args(rp.myargv()[1:])

    slam_bag = BagOrder(args.bag_file, args.out_path)

    if not args.ref_topic:
        # if no reference topic is specified, take first topic that is contained in bagfile
        args.ref_topic = slam_bag.topics[0]

    slam_bag.set_param(args.ref_topic, args.axis, args.offset, args.mirror)
    slam_bag.read_array_data(args.ref_topic, args.out_path)


