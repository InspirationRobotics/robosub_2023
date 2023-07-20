#!/usr/bin/env python
import os
import signal
import threading
import time
from datetime import datetime

import rosbag
import rospy
import std_msgs.msg
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import FluidPressure, Imu


class rosBags:
    def create_rosbag(self):
        rospy.init_node("rosbag_creator", anonymous=True)
        fileName = str(datetime.now())
        fileName = fileName.split(".")
        fileName = fileName[0].split(" ")
        temp = fileName[1].split(":")
        fileName[1] = f"{temp[0]}-{temp[1]}-{temp[2]}"
        fileName = f"{fileName[0]}_{fileName[1]}"
        self.name = f"/home/inspiration/bags/{fileName}.bag"
        os.system("mkdir /home/inspiration/bags >/dev/null 2>&1")
        os.system(f"touch {self.name}")
        self.bag = rosbag.Bag(self.name, "w")
        print(f"Successfully made bag file: {self.name}")

        # Subscribe to the desired topics and save messages to the bag
        rospy.Subscriber("/auv/devices/compass", std_msgs.msg.Float64, self.bag_write_callback, callback_args=("/auv/devices/compass"))
        rospy.Subscriber("/auv/devices/imu", Imu, self.bag_write_callback, callback_args=("/auv/devices/imu"))
        rospy.Subscriber("/auv/devices/baro", std_msgs.msg.Float32MultiArray, self.bag_write_callback, callback_args=("/auv/devices/baro"))
        rospy.Subscriber("/auv/devices/thrusters", OverrideRCIn, self.bag_write_callback, callback_args=("/auv/devices/thrusters"))
        rospy.Subscriber("/auv/devices/setDepth", std_msgs.msg.Float64, self.bag_write_callback, callback_args=("/auv/devices/setDepth"))
        rospy.Subscriber("/auv/status/arm", std_msgs.msg.Bool, self.bag_write_callback, callback_args=("/auv/status/arm"))
        rospy.Subscriber("/auv/status/mode", std_msgs.msg.String, self.bag_write_callback, callback_args=("/auv/status/mode"))
        rospy.spin()

    def closeBag(self):
        # Close the bag file
        self.bag.close()

    def bag_write_callback(self, msg, topic):
        self.bag.write(topic, msg)


def onExit(signum, frame):
    try:
        print("\nClosing Rosbag...")
        rospy.signal_shutdown("Rospy Exited")
        while not rospy.is_shutdown():
            pass
        bag.closeBag()
        time.sleep(1)
        print(f"\nBag Saved to {bag.name} ...Done")
        exit(1)
    except:
        pass


signal.signal(signal.SIGINT, onExit)

if __name__ == "__main__":
    bag = rosBags()
    bag.create_rosbag()
