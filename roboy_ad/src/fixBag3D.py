#!/usr/bin/env python

import rosbag as rb
import genpy as gp
import sys
import os
import time
import argparse


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

            self._start_time = self.bag.get_start_time()  # store start time of messages
            self._duration = self.bag.get_end_time() - self._start_time  # store duration
            self._topic_timerange = {}
	    
	    if os.path.isdir(outpath):
		filename, ext = os.path.splitext(os.path.split(self.file)[1])
                self.sorted_file = outpath + filename + '_sort' + ext
                self.sorted_bag = rb.Bag(self.sorted_file, 'w')
	    else:
                root, ext = os.path.splitext(self.file)  # make path and bag for sorted messages
                self.sorted_file = root + '_sort' + ext
                self.sorted_bag = rb.Bag(self.sorted_file, 'w')

        else:
            print("IOError: Argument path does not point to a valid bag file.")
            print("Make sure, that a valid path to a bag file (*.bag) is passed to this class.")

    def get_topic_timerange(self, bag):
        """
        For each relevant topic in the bag file, earliest and latest time stamp plus duration are determined
        for both, message and header time, and stored in a dictionary

        Output:
        - topic_timerange (dict)
            * topic (dict)
                * header_timestamp (dict)
                    * start (genpy.rostime.Time)
                    * end (genpy.rostime.Time)
                    * duration (float)
                * msg_timestamp (dict)
                    * start (genpy.rostime.Time)
                    * end (genpy.rostime.Time)
                    * duration (float)
        """

        time_init = gp.rostime.Time.from_sec(0)  # time only used as element of comparison
        dict_template = {'start': time_init, 'end': time_init}
        self._topic_timerange = dict(zip(self.topics,  # make dictionary for all time stamps
                                         [{'msg_timestamp': dict_template.copy(),
                                           'header_timestamp': dict_template.copy()}.copy() for _ in self.topics]))

        # iterate through all messages in the bag
        for topic, msg, t in bag.read_messages(topics=['/imu', '/points2', '/imu_data', '/livox/lidar']):

            # check message time stamps for new earliest or latest time stamp
            # check whether new earliest message
            if self._topic_timerange[topic]['msg_timestamp']['start'] == time_init \
                    or t < self._topic_timerange[topic]['msg_timestamp']['start']:
                self._topic_timerange[topic]['msg_timestamp']['start'] = t
            # check whether new latest message
            if self._topic_timerange[topic]['msg_timestamp']['end'] == time_init \
                    or t > self._topic_timerange[topic]['msg_timestamp']['end']:
                self._topic_timerange[topic]['msg_timestamp']['end'] = t

            # check header time stamps for new earliest or latest time stamp
            if msg._has_header:
                t_header = msg.header.stamp
                # check whether new earliest message
                if self._topic_timerange[topic]['header_timestamp']['start'] == time_init \
                        or t_header < self._topic_timerange[topic]['header_timestamp']['start']:
                    self._topic_timerange[topic]['header_timestamp']['start'] = t_header
                # check whether new latest message
                if self._topic_timerange[topic]['header_timestamp']['end'] == time_init \
                        or t_header > self._topic_timerange[topic]['header_timestamp']['end']:
                    self._topic_timerange[topic]['header_timestamp']['end'] = t_header

        # all topics
        for topic in self._topic_timerange.keys():
            # both message and header time stamp
            for stamp_type in ['msg_timestamp', 'header_timestamp']:
                self._topic_timerange[topic][stamp_type]['duration'] \
                    = (self._topic_timerange[topic][stamp_type]['end']
                       - self._topic_timerange[topic][stamp_type]['start']).to_sec()  # compute time range (duration)

        return self._topic_timerange

    def reorder_bag(self, reference_topic):
        """
        Reorders the classes bag file so that afterwards all messages appear in chronological order.
        Writes sorted messages into new bagfile

        Output:
        - bagfile_reordered (str): path to file (*.bag) in which messages are ordered chronologically
        """

        print('Reordering Messages...')

        if reference_topic not in self.topics:
            print('The selected reference topic is not one of the topics contained in the bag file.')
            print('You can see which topics are present in the bag file, calling \"rosbag info <path/to/bagfile>\" '
                  'from your terminal')
            return -1

        # determine range of time for the different topics and time stamps
        if not self._topic_timerange:
            topic_timerange = self.get_topic_timerange(self.bag)
        else:
            topic_timerange = self._topic_timerange

        # for each topic determine time displacement that needs to be added to header time stamp
        offset_times = {}
        for topic in self.topics:
            offset = topic_timerange[reference_topic]['header_timestamp']['start'].to_sec() \
                     - topic_timerange[topic]['header_timestamp']['start'].to_sec() \
                     + topic_timerange[topic]['msg_timestamp']['start'].to_sec() \
                     - topic_timerange[reference_topic]['msg_timestamp']['start'].to_sec()
            offset_times[topic] = offset

        # write new bag
        with self.sorted_bag as newbag:

            last_time = time.clock()

            # iterate over all messages
            for topic, msg, t in self.bag.read_messages(topics=['/imu', '/points2', '/imu_data', '/livox/lidar']):
		
		if topic == '/livox/lidar':
		    newtopic = '/points2'
		else:
		    newtopic = topic
		
                # update status bar every 100 ms
                if time.clock() - last_time > .1:
                    percent = (t.to_sec() - self._start_time) / self._duration
                    status(40, percent)
                    last_time = time.clock()

                # time stamp overwrite only works for messages with header time stamp
                if msg._has_header:
                    # add displacement time to header time stamp
                    new_time = gp.rostime.Time.from_sec(msg.header.stamp.to_sec() + offset_times[topic])

                    # update both, header and message time stamp
                    msg.header.stamp = new_time
                    newbag.write(newtopic, msg, new_time)
                else:
                    print('Reordering only works for messages with separate time stamp in message header.')
                    print('Message of topic {} and with time stamp {} could not be reordered'.format(topic, t))
                    newbag.write(newtopic, msg, t)

        status(40, 1)
        print('\nfinished successfully')

        return self.sorted_file


if __name__ == '__main__':

    parser = argparse.ArgumentParser('Reorder an existing bag file')
    parser.add_argument('bag_file', help='path to the bagfile whose messages should be reordered')
    parser.add_argument('--ref_topic', '-rf', help='topic name which acts as a reference for the time stamps')
    parser.add_argument('--out_path', '-o', help='path to file with reordered bag file')
    args = parser.parse_args()

    slam_bag = BagOrder(args.bag_file, args.out_path)

    if args.ref_topic is None:
        # if no reference topic is specified, take first topic that is contained in bagfile
        args.ref_topic = slam_bag.topics[0]

    new_file = slam_bag.reorder_bag(args.ref_topic)
