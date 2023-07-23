#!/usr/bin/env python2

import os
import signal
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from . import pyfakewebcam

preDevices = os.popen("ls /dev/video*").read()
os.system("sudo modprobe v4l2loopback devices=2")
postDevices = os.popen("ls /dev/video*").read()
diff = postDevices[len(preDevices) :]
diff = diff.split("\n")
newDevice1 = diff[0]
newDevice2 = diff[1]
preDevices = preDevices.split("\n")
ogDevice1 = preDevices[0]
ogDevice2 = preDevices[2]

IMG_W = 640
IMG_H = 480

cam1 = cv2.VideoCapture(ogDevice1)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
cam2 = cv2.VideoCapture(ogDevice2)
cam2.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)


fake1 = pyfakewebcam.FakeWebcam(newDevice1, IMG_W, IMG_H)
fake2 = pyfakewebcam.FakeWebcam(newDevice2, IMG_W, IMG_H)
print(f"Forward output at: {newDevice1}")
print(f"Downward output at: {newDevice2}")


class CameraStreams:
    def __init__(self):
        # Params
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
        try:
            self.forwardVideo = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
            fake1.schedule_frame(self.forwardVideo)
        except Exception as e:
            print("Forward Output Error, make sure running in Python2")
            print(e)

    def callbackBottom(self, msg):
        try:
            self.bottomVideo = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
            fake2.schedule_frame(self.bottomVideo)
        except Exception as e:
            print("Bottom Output Error, make sure running in Python2")
            print(e)

    def start(self):
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


signal.signal(signal.SIGINT, onExit)

if __name__ == "__main__":
    rospy.init_node("CameraStream", anonymous=True)
    my_node = CameraStreams()
    my_node.start()

# https://youtu.be/2l913YwWYe4
# https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros

# At the top of pyfakewebcam.py and __init__.py add "from __future__ import absolute_import"
# path: /home/inspiration/.local/lib/python2.7/site-packages/pyfakewebcam
