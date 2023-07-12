import os
import platform
import signal
import sys
import threading
import time

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from . import pyfakewebcam

class USBCamera:
    def __init__(self, rospy, id, ogDevice, newDevice):
        IMG_W = 640
        IMG_H = 480
        self.rospy = rospy
        self.frame = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(30)
        self.id = id
        self.cam = cv2.VideoCapture(ogDevice)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
        self.fake = pyfakewebcam.FakeWebcam(newDevice, IMG_W, IMG_H)
        self.pub = self.rospy.Publisher("/auv/camera/videoUSBRaw" + str(id), Image, queue_size=10)
        self.rospy.Subscriber("/auv/camera/videoUSBOutput" + str(id), Image, self.callbackMain)
        self.time = time.time()
        print(ogDevice + " is available at " + newDevice)

    def callbackMain(self, msg):
        self.time = time.time()
        self.sendFakeFrame(self.br.imgmsg_to_cv2(msg))

    def sendFakeFrame(self, msg):
        try:
            self.frame = cv2.cvtColor(msg, cv2.COLOR_BGR2RGB)
            self.fake.schedule_frame(self.frame)
        except Exception as e:
            print("Camera " + str(self.id) + " Output Error, make sure running in correct python")
            print(e)

    def runner(self):
        while not self.rospy.is_shutdown():
            try:
                ret, frame1 = self.cam.read()
                msg = self.br.cv2_to_imgmsg(frame1)
                self.pub.publish(msg)
                if(time.time()-self.time>3): #no new CV output frames recieved, default to cam view
                        self.sendFakeFrame(frame1)
                pass
            except Exception as e:
                print("Camera " + str(self.id) + " Input Error")
                print(e)
        self.loop_rate.sleep()

    def kill(self):
        self.cam.release()

    def start(self):
        self.rospy.loginfo("Starting Camera " + str(self.id) + " Stream...")
        self.thread_param_updater = threading.Timer(0, self.runner)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()