#!/usr/bin/env python

import rospy
import rosbag


def create_rosbag(bag_filename):
    rospy.init_node('rosbag_creator', anonymous=True)
    bag = rosbag.Bag('dl.bag', 'w')

    # Subscribe to the desired topics and save messages to the bag
    subscribers = []
    subscribers.append(rospy.Subscriber(topic, String, bag_write_callback, callback_args=(bag, topic)))

    rospy.spin()

    # Close the bag file
    bag.close()


def bag_write_callback(msg, args):
    bag = args[0]
    topic = args[1]
    bag.write(topic, msg)


if __name__ == '__main__':
    create_rosbag()
