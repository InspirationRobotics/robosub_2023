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

from auv.device.cams import pyfakewebcam


class USBCamera:
    def __init__(self, rospy, id, ogDevice, newDevice):
        self.IMG_W = 640
        self.IMG_H = 480
        self.rospy = rospy
        self.id = id
        self.frame = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(30)
        self.isKilled = True
        self.ogDevice = ogDevice
        self.newDevice = newDevice
        self.fake = pyfakewebcam.FakeWebcam(newDevice, self.IMG_W, self.IMG_H)
        self.pub = self.rospy.Publisher(f"/auv/camera/videoUSBRaw{str(id)}", Image, queue_size=10)
        self.rospy.Subscriber(f"/auv/camera/videoUSBOutput{str(id)}", Image, self.callbackMain)
        self.time = time.time()
        print(f"Camera ID {str(id)}: {ogDevice} is available at {newDevice}")

    def callbackMain(self, msg):
        if self.isKilled:
            return
        self.time = time.time()
        self.sendFakeFrame(self.br.imgmsg_to_cv2(msg))

    def sendFakeFrame(self, msg):
        try:
            self.frame = cv2.cvtColor(msg, cv2.COLOR_BGR2RGB)
            self.fake.schedule_frame(self.frame)
        except Exception as e:
            print(f"Camera {str(self.id)} Output Error, make sure running in correct python")
            print(e)

    def runner(self):
        while not self.rospy.is_shutdown() and not self.isKilled:
            try:
                ret, frame1 = self.cam.read()
                if ret:
                    msg = self.br.cv2_to_imgmsg(frame1)
                    self.pub.publish(msg)
                    if time.time() - self.time > 3:  # no new CV output frames recieved, default to cam view
                        self.sendFakeFrame(frame1)
                pass
            except Exception as e:
                print(f"Camera {str(self.id)} Input Error")
                print(e)
        self.loop_rate.sleep()

    def kill(self):
        if self.isKilled:
            return
        self.rospy.loginfo(f"Killing Camera {str(self.id)} Stream...")
        self.isKilled = True
        self.usbThread.join()
        self.cam.release()
        self.rospy.loginfo(f"Killed Camera {str(self.id)} Stream...")

    def start(self):
        self.cam = cv2.VideoCapture(self.ogDevice)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.IMG_W)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.IMG_H)
        self.isKilled = False
        self.rospy.loginfo(f"Starting Camera {str(self.id)} Stream...")
        self.usbThread = threading.Timer(0, self.runner)
        self.usbThread.daemon = True
        self.usbThread.start()
