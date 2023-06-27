#!/usr/bin/env python
import rospy
import rosbag
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import OverrideRCIn, State
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Float64
import threading
import time
import datetime

def create_rosbag():
    rospy.init_node('rosbag_creator', anonymous=True)
    name = '~/bags/' + str(datetime.now())
    bag = rosbag.Bag(name, 'w')

    # Subscribe to the desired topics and save messages to the bag
    rospy.Subscriber("/auv/devices/compass", Float64, bag_write_callback, callback_args=(bag, "/auv/devices/compass"))
    rospy.Subscriber("/auv/devices/imu", Imu, bag_write_callback, callback_args=(bag, "/auv/devices/imu"))
    rospy.Subscriber("/auv/devices/baro", std_msgs.msg.Float32MultiArray, bag_write_callback, callback_args=(bag, "/auv/devices/baro"))
    rospy.Subscriber("/auv/devices/thrusters", OverrideRCIn, bag_write_callback, callback_args=(bag, "/auv/devices/thrusters"))
    rospy.Subscriber("/auv/devices/setDepth", Float64, bag_write_callback, callback_args=(bag, "/auv/devices/setDepth"))
    rospy.Subscriber("/auv/status/arm", std_msgs.msg.Bool, bag_write_callback, callback_args=(bag, "/auv/status/arm"))
    rospy.Subscriber("/auv/status/mode", std_msgs.msg.String, bag_write_callback, callback_args=(bag, "/auv/status/mode"))
    rospy.spin()

    # Close the bag file
    bag.close()


def bag_write_callback(msg, args):
    bag = args[0]
    topic = args[1]
    bag.write(topic, msg)


if __name__ == '__main__':
    create_rosbag()
