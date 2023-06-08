#!/usr/bin/env python2

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import time
import pyfakewebcam
import numpy as np
import signal
import threading

ogDev = []
newDevice = []
cam = []
fake = []
preDevices = os.popen('ls /dev/video*').read()
preDevices = preDevices.split("\n")
for i in len(preDevices):
    if(i%2==0):
        ogDev.append(preDevices[i])
os.system('sudo modprobe v4l2loopback devices='+len(ogDev))
postDevices = os.popen('ls /dev/video*').read()
diff = postDevices[len(preDevices):]
diff = diff.split("\n")
for i in len(diff):
    newDevice.append(diff[i])


IMG_W = 640
IMG_H = 480

for i in len(preDevices):
    cam.append(cv2.VideoCapture(ogDev[i]))
    cam[i].set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
    cam[i].set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
    fake.append(pyfakewebcam.FakeWebcam(newDevice[i], IMG_W, IMG_H))
    print(ogDev[i] + " output at " + newDevice[i])

class camera():
    def __init__(self, id):
        self.frame = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(30)
        self.id = id

        self.pub = rospy.Publisher('/auv/camera/videoRaw'+str(id), Image,queue_size=10)
        rospy.Subscriber("/auv/camera/videoOutput"+str(id),Image,self.callbackMain)

    def callbackMain(self, msg):
        try:
            self.frame = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
            fake[self.id].schedule_frame(self.frame)
        except Exception as e:
            print("Camera "+self.id+" Output Error, make sure running in Python2")
            print(e)
    
    def runner(self):
        while not rospy.is_shutdown():
            try:
                ret, frame1 = cam[self.id].read()
                self.pub.publish(self.br.cv2_to_imgmsg(frame1))
            except Exception as e:
                print("Camera "+self.id+" Input Error")
                print(e)

            self.loop_rate.sleep()

    def start(self):
        rospy.loginfo("Starting Camera "+self.id+" Stream...")
        self.thread_param_updater = threading.Timer(0, self.runner)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()

class cameraStreams():
    def __init__(self):
        rospy.loginfo("Initializing Camera Streams...")
        self.cams == []
        for i in len(preDevices):
            self.cams.append(camera(i))
        
    def start(self):
        for i in self.cams:
            i.start()
            
def onExit(signum, frame):
    try:
        print("\Closing Cameras and exiting...")
        for i in cam:
            i.release()
        time.sleep(1)
        rospy.signal_shutdown("Rospy Exited")
        while not rospy.is_shutdown():
            pass
        print("\n\nCleanly Exited")
        print('Please run: \nsudo modprobe -r v4l2loopback\n')
        exit(1)
    except:
        pass

signal.signal(signal.SIGINT, onExit)

if __name__ == '__main__':
    rospy.init_node("CameraStream", anonymous=True)
    my_node = cameraStreams()
    my_node.start()

#https://youtu.be/2l913YwWYe4
#https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros

#At the top of pyfakewebcam.py and __init__.py add "from __future__ import absolute_import"
#path: /home/inspiration/.local/lib/python2.7/site-packages/pyfakewebcam
