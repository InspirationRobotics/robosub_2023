"""
Deprecated method for capturing video streams from camera and publishing them to ROS topics
"""

#!/usr/bin/env python2

import os
import signal
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image # ROS message type for camera stream (1 image per frame)

from . import pyfakewebcam # Class for writing frames to a virtual video device to simulate a webcam

# Read the configuration of all the avaliable video devices
preDevices = os.popen("ls /dev/video*").read()

# Loads the V4L2 video device with the option to create two different feeds
os.system("sudo modprobe v4l2loopback devices=2")

# Read the configuration of all the avaliable video devices
postDevices = os.popen("ls /dev/video*").read()

# Find the list of devices that were newly added since the V4L2 video device was created
diff = postDevices[len(preDevices) :]

# Find the newly added devices (these are the two devices allowed in "sudo modprobe v4l2loopback devices = 2"), and split them by entry so that each device has their own configuration
diff = diff.split("\n")
newDevice1 = diff[0]
newDevice2 = diff[1]

# Assign the previously already loaded devices (these are actual real cameras) to their respective paths, so that each device has their own video path (configuration)
preDevices = preDevices.split("\n")
ogDevice1 = preDevices[0]
ogDevice2 = preDevices[2]

# Set image width, height
IMG_W = 640
IMG_H = 480

# Capture the video of the actual cameras
cam1 = cv2.VideoCapture(ogDevice1)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
cam2 = cv2.VideoCapture(ogDevice2)
cam2.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)

# Create a simulated webcam for the "new" devices
fake1 = pyfakewebcam.FakeWebcam(newDevice1, IMG_W, IMG_H)
fake2 = pyfakewebcam.FakeWebcam(newDevice2, IMG_W, IMG_H)

# Two streams -- one facing forward, one facing downward
print(f"Forward output at: {newDevice1}")
print(f"Downward output at: {newDevice2}")


class CameraStreams:
    """
    Creates camera streams for the real and fake devices

    NOTE: "Devices" in this context are all cameras
    """
    def __init__(self):
        # Parameters
        self.forwardVideo = None
        self.bottomVideo = None
        self.br = CvBridge()

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(30)

        # Publishers
        self.pubForward = rospy.Publisher("/auv/camera/videoForwardRaw", Image, queue_size=10)
        self.pubBottom = rospy.Publisher("/auv/camera/videoBottomRaw", Image, queue_size=10)

        # Subscribers
        rospy.Subscriber("/auv/camera/videoForwardOutput", Image, self.callbackForward)
        rospy.Subscriber("/auv/camera/videoBottomOutput", Image, self.callbackBottom)

    def callbackForward(self, msg):
        """
        For the real devices, this method changes each image format so that it can be captured as a camera stream by OpenCV
        For the fake devices, this method creates the camera stream directly

        This method only deals with forward-facing devices

        Args:
            msg (numpy.ndarray): Frame to be converted (comes as a BGR image for the real device, for the fake device it should already be in RGB)
        """
        try:
            self.forwardVideo = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB) # Convert from BGR to RGB for the real device
            fake1.schedule_frame(self.forwardVideo) # Start the camera stream for the fake device
        except Exception as e:
            print("Forward Output Error, make sure running in Python2")
            print(e)

    def callbackBottom(self, msg):
        """
        For the real devices, this method changes each image format so that it can be captured as a camera stream by OpenCV
        For the fake devices, this method creates the camera stream directly

        This method only deals with downward-facing devices

        Args:
            msg (numpy.ndarray): Frame to be converted (comes as a BGR image for the real device, for the fake device it should already be in RGB)
        """
        try:
            self.bottomVideo = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB) # Convert from BGR to RGB for the real device
            fake2.schedule_frame(self.bottomVideo) # Start the camera stream for the fake device
        except Exception as e:
            print("Bottom Output Error, make sure running in Python2")
            print(e)

    def start(self):
        """
        Start the video streams for the real camera streams
        """
        # Capture and read the frame, then publish to the ROS topic
        rospy.loginfo("Starting Video Streams...")
        while not rospy.is_shutdown():
            try:
                ret, frame1 = cam1.read()
                self.pubForward.publish(self.br.cv2_to_imgmsg(frame1))
            except Exception as e:
                print("Forward Input Error")
                print(e)
            try:
                ret, frame2 = cam2.read()
                self.pubBottom.publish(self.br.cv2_to_imgmsg(frame2))
            except Exception as e:
                print("Forward Input Error")
                print(e)

            self.loop_rate.sleep()


def onExit(signum, frame):
    """
    Exits the camera streams gracefully

    Args:
        signum (int): the number indicating the signal that was sent (Ctrl + C = 2)
        frame (numpy.ndarray): the current frame that was being read when the signal was received to exit (Ctrl + C)
    """
    try:
        print("\Closing Cameras and exiting...")
        cam1.release()
        cam2.release()
        time.sleep(1)
        rospy.signal_shutdown("Rospy Exited")
        while not rospy.is_shutdown():
            pass
        print("\n\nCleanly Exited")
        print("Please run: \nsudo modprobe -r v4l2loopback\n")
        exit(1)
    except:
        pass

# Handles keyboard interrupt -- starts the onExit function when Ctrl + C is pressed
signal.signal(signal.SIGINT, onExit)

# If called directly, start the camera stream by initializing a ROS node then calling the class
if __name__ == "__main__":
    rospy.init_node("CameraStream", anonymous=True)
    my_node = CameraStreams()
    my_node.start()

# https://youtu.be/2l913YwWYe4
# https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros

# At the top of pyfakewebcam.py and __init__.py add "from __future__ import absolute_import"
# path: /home/inspiration/.local/lib/python2.7/site-packages/pyfakewebcam
