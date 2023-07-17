import ctypes #comment out for Onyx
libgcc_s = ctypes.CDLL('libgcc_s.so.1') #comment out for Onyx
import os
import platform
import signal
import sys
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from auv.device.cams import pyfakewebcam, usbCams, oakCams
from auv.utils import deviceHelper

if sys.version_info[0] == 3:
    # python3 meaning oak-d
    import depthai as dai

# order is forward, down
onyx = [deviceHelper.dataFromConfig("forwardUSB")]
grey = [deviceHelper.dataFromConfig("forwardUSB"), deviceHelper.dataFromConfig("bottomUSB")]
ogDev = []
oaks = []
newDev = []
sub = False  # default to grey


def list_devices():
    available_devices = []
    for device in dai.Device.getAllAvailableDevices():
        available_devices.append(device.getMxId())
    return available_devices

def difference(string1, string2):
    string1 = string1.split()
    string2 = string2.split()
    A = set(string1)
    B = set(string2)
    str_diff = A.symmetric_difference(B)
    return list(str_diff)


preDevices = os.popen("ls /dev/video*").read()
if "nx" in platform.node():
    sub = True
    ogDev = deviceHelper.findCam(onyx)
else:
    ogDev = deviceHelper.findCam(grey)

oaks = list_devices()
oakAmt = len(oaks)
print(oaks)
camAmt = len(ogDev)
print(ogDev)
os.system("sudo modprobe v4l2loopback devices=" + str(camAmt + oakAmt))
postDevices = os.popen("ls /dev/video*").read()
diff = difference(preDevices, postDevices)
if len(diff) == 0:
    print("Failed to detect if any new v4l2loopback devices were made")
    exit(1)
for i in range(camAmt + oakAmt - 1, -1, -1):
    newDev.append(diff[i])

# v4l2-ctl --list-devices

class cameraStreams:
    def __init__(self):
        rospy.loginfo("Initializing Camera Streams...")
        self.cams = []
        for i in range(camAmt):
            self.cams.append(usbCams.USBCamera(rospy, i, ogDev[i], newDev[i]))
        for i in range(oakAmt):
            self.cams.append(oakCams.oakCamera(rospy, i + camAmt, oaks[i], newDev[i + camAmt]))

    def start(self):
        while True:
            stream = input("Select Camera stream to start or stop\n")
            print(stream+" is selected. Enter 'start' or 'stop' or 'back' to pick different camera\n")
            cmd = input()
            if(cmd=="start"):
                self.cams[int(stream)].start()
            elif(cmd=="stop"):
                self.cams[int(stream)].kill()
            elif(cmd=="back"):
                continue

    def stop(self):
        for i in self.cams:
            i.kill()
            del i

def onExit(signum, frame):
    try:
        print("\Closing Cameras and exiting...")
        my_node.stop()
        time.sleep(3)
        rospy.signal_shutdown("Rospy Exited")
        while not rospy.is_shutdown():
            pass
        print("\n\nCleanly Exited")
        print("Please run: \nsudo modprobe -r v4l2loopback\n")
        exit(1)
    except:
        pass

signal.signal(signal.SIGINT, onExit)

if __name__ == "__main__":
    rospy.init_node("CameraStream", anonymous=True)
    my_node = cameraStreams()
    my_node.start()

# https://youtu.be/2l913YwWYe4
# https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros

# At the top of pyfakewebcam.py and __init__.py add "from __future__ import absolute_import"
# path: /home/inspiration/.local/lib/python2.7/site-packages/pyfakewebcam
