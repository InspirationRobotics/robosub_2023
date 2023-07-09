import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import time
import cams.pyfakewebcam as pyfakewebcam
import numpy as np
import signal
import threading
import platform
from cams.camsHelper import findCam
import sys

if sys.version_info[0] == 3:
    #python3 meaning oak-d
    import depthai as dai

#order is forward, down
onyx = ["platform-3610000.xhci-usb-0:2.2.4:1.0"]
grey = ["platform-70090000.xusb-usb-0:2.3:1.0","platform-70090000.xusb-usb-0:2.4:1.0"]

ogDev = []
oaks = []
newDev = []
sub = False #default to grey

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

preDevices = os.popen('ls /dev/video*').read()
if("nx" in platform.node()):
    sub  = True
    ogDev = findCam(onyx)
    oaks = list_devices()
    oakAmt = len(oaks)
    print(oaks)
else:
    ogDev = findCam(grey)
    oakAmt=0
camAmt = len(ogDev)
print(ogDev)
os.system('sudo modprobe v4l2loopback devices='+str(camAmt+oakAmt))
postDevices = os.popen('ls /dev/video*').read()
diff = difference(preDevices, postDevices)
if(len(diff)==0):
    print("Failed to detect if any new v4l2loopback devices were made")
    exit(1)
for i in range(camAmt+oakAmt-1, -1, -1):
    newDev.append(diff[i])

#v4l2-ctl --list-devices

# for i in range(camAmt):
#     cam.append(cv2.VideoCapture(ogDev[i]))
#     cam[i].set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
#     cam[i].set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
#     fake.append(pyfakewebcam.FakeWebcam(newDevice[i], IMG_W, IMG_H))
#     print(ogDev[i] + " output at " + newDevice[i])

class camera():
    def __init__(self, id, ogDevice, newDevice, oak=False):
        IMG_W = 640
        IMG_H = 480
        self.frame = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(30)
        self.id = id
        self.oak = oak
        if(not self.oak):
            self.cam = cv2.VideoCapture(ogDevice)
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
            self.fake = pyfakewebcam.FakeWebcam(newDevice, IMG_W, IMG_H)
            self.pub = rospy.Publisher('/auv/camera/videoUSBRaw'+str(id), Image,queue_size=10)
            rospy.Subscriber("/auv/camera/videoUSBOutput"+str(id),Image,self.callbackMain)
            print(ogDevice+" is available at " + newDevice)
        else:
            self.name = self.mxidToName(ogDevice)
            pipeline = dai.Pipeline()
            xoutRgb = pipeline.createXLinkOut()
            xoutRgb.setStreamName("rgb")
            controlIn = pipeline.create(dai.node.XLinkIn)
            controlIn.setStreamName('control')
            camRgb = pipeline.createColorCamera()
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setIspScale(2,3)
            camRgb.setPreviewSize(640, 480)
            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            camRgb.preview.link(xoutRgb.input)
            controlIn.out.link(camRgb.inputControl)
            device_info = dai.DeviceInfo(ogDevice)
            self.device = dai.Device(pipeline, device_info)
            controlQueue = self.device.getInputQueue('control')
            # Autofocus trigger (and disable continuous)
            ctrl = dai.CameraControl()
            ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
            ctrl.setAutoFocusTrigger()
            controlQueue.send(ctrl)

            # pipeline = dai.Pipeline()
            # camRgb = pipeline.createColorCamera()
            # xoutRgb = pipeline.createXLinkOut()
            # xoutRgb.setStreamName("rgb")
            # camRgb.preview.link(xoutRgb.input)
            # device_info = dai.DeviceInfo(ogDevice)
            # self.device = dai.Device(pipeline, device_info)

            self.fake = pyfakewebcam.FakeWebcam(newDevice, IMG_W, IMG_H)
            self.pub = rospy.Publisher('/auv/camera/videoOAKdRaw'+self.name, Image,queue_size=10)
            rospy.Subscriber("/auv/camera/videoOAKd"+self.name+"Model",Image,self.callbackModel)
            rospy.Subscriber('/auv/camera/videoOAKdRaw'+self.name,Image,self.callbackMain)
            print("Oak-D " + self.name + " is available at " + newDevice)

    def mxidToName(self, mxid):
        if(mxid=='18443010B1A0B01200'):
            return 'Forward'
        elif(mxid=='184430104161721200'):
            return 'Bottom'
        elif(mxid=='18443010D11AF50F00'):
            return 'Poe'
        else:
            return 'Unknown'

    def callbackMain(self, msg):
        try:
            self.frame = cv2.cvtColor(self.br.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
            self.fake.schedule_frame(self.frame)
        except Exception as e:
            print("Camera "+str(self.id)+" Output Error, make sure running in correct python")
            print(e)
    
    def callbackModel(self, msg):
        pass #need to implment blob file switching
    
    def runner(self):
        while not rospy.is_shutdown():
            if(not self.oak):
                try:
                    ret, frame1 = self.cam.read()
                    msg = self.br.cv2_to_imgmsg(frame1)
                    self.pub.publish(msg)
                    pass
                except Exception as e:
                    print("Camera "+str(self.id)+" Input Error")
                    print(e)
            else:
                cam = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                try:
                    frame1 = cam.get().getCvFrame()
                    msg = self.br.cv2_to_imgmsg(frame1)
                    self.pub.publish(msg)
                    pass
                except Exception as e:
                    print("Camera "+str(self.id)+" Input Error")
                    print(e)
            self.loop_rate.sleep()

    def kill(self):
        if(not self.oak):
            self.cam.release()

    def start(self):
        rospy.loginfo("Starting Camera "+str(self.id)+" Stream...")
        self.thread_param_updater = threading.Timer(0, self.runner)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()

class cameraStreams():
    def __init__(self):
        rospy.loginfo("Initializing Camera Streams...")
        self.cams = []
        self.oakCams = []
        for i in range(camAmt):
            self.cams.append(camera(i, ogDev[i], newDev[i]))
        for i in range(oakAmt-1):
            self.oakCams.append(camera(i+camAmt, oaks[i], newDev[i+camAmt], True))

    def start(self):
        for i in self.cams:
            i.start()
        for i in self.oakCams:
            i.start()
        rospy.spin()

    def stop(self):
        for i in self.cams:
            i.kill()

def onExit(signum, frame):
    try:
        print("\Closing Cameras and exiting...")
        my_node.stop()
        time.sleep(3)
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
