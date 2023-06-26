#!/usr/bin/env python
import rospy
import rosbag
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import OverrideRCIn, State
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Float64
import threading
import time


def create_rosbag(bag_filename):
    rospy.init_node('rosbag_creator', anonymous=True)
    bag = rosbag.Bag('dl.bag', 'w')

    # Subscribe to the desired topics and save messages to the bag
    rospy.Subscriber("/mavros/state", State, bag_write_callback, callback_args=(bag, "/mavros/state"))
    rospy.Subscriber("/mavros/imu/data, Imu, bag_write_callback, callback_args=(bag, "/mavros/imu/data"))
    rospy.Subscriber("/mavros/imu/FluidPressure", FluidPressure, bag_write_callback, callback_args=(bag, "/mavros/imu/FluidPressure"))
    rospy.Subscriber("/mavros/rc/override", OvverrideRCIn, bag_write_callback, callback_args=(bag, "/mavros/rc/override"))
    rospy.spin()

    # Close the bag file
    bag.close()


def bag_write_callback(msg, args):
    bag = args[0]
    topic = args[1]
    bag.write(topic, msg)


if __name__ == '__main__':
    create_rosbag()
