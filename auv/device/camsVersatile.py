import lsb_release

if lsb_release.get_distro_information()["RELEASE"] == "18.04":
    import ctypes

    libgcc_s = ctypes.CDLL("libgcc_s.so.1")
import os
import platform
import signal
import sys
import time
import rospy
import json
import re
import depthai as dai
from std_msgs.msg import String
from auv.device.cams import pyfakewebcam, usbCams, oakCams
from auv.utils import deviceHelper


# order is forward, down
usbIDS = [deviceHelper.dataFromConfig("forwardUSB"), deviceHelper.dataFromConfig("bottomUSB")]
ogDev = []
oaks = []
newDev = []


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


def num_sort(test_string):
    return list(map(int, re.findall(r"\d+", test_string)))[0]


preDevices = os.popen("ls /dev/video*").read()
ogDev = deviceHelper.findCam(usbIDS)
oaks = list_devices()
oakAmt = len(oaks)
print(oaks)
camAmt = len(ogDev)
print(ogDev)
os.system(f"sudo modprobe v4l2loopback devices={str(camAmt + oakAmt)}")
postDevices = os.popen("ls /dev/video*").read()
diff = difference(preDevices, postDevices)
if len(diff) == 0:
    print("Failed to detect if any new v4l2loopback devices were made")
    exit(1)
for i in range(camAmt + oakAmt - 1, -1, -1):
    newDev.append(diff[i])

newDev.sort(key=num_sort)

# v4l2-ctl --list-devices


class cameraStreams:
    def __init__(self, debug=False):
        rospy.loginfo("Initializing Camera Streams...")
        self.cams = []
        self.camID = []
        self.activeCams = []
        self.debug = debug
        self.limit = 2  # max amount of cams that can be used at a time
        for i in range(camAmt):
            self.cams.append(usbCams.USBCamera(rospy, i, ogDev[i], newDev[i]))
            self.camID.append(i)
        for i in range(oakAmt):
            self.cams.append(oakCams.oakCamera(rospy, i + camAmt, oaks[i], newDev[i + camAmt]))
            self.camID.append(i + camAmt)
        rospy.Subscriber("/auv/camsVersatile/cameraSelect", String, self.callbackCamSelect)
        if debug:
            print("*****************\ndebug mode is ON\n*****************")

    def start(self):
        while self.debug and not rospy.is_shutdown():
            stream = input("Select Camera ID to control\n")
            print(
                "Camera "
                + stream
                + " is selected. Enter 'start' or 'stop' or 'model' to pick a model to run or 'back' to pick different camera \n"
            )
            cmd = input()
            if cmd == "start":
                self.camCtrl(stream, True)
            elif cmd == "stop":
                self.camCtrl(stream, False)
            elif cmd == "back":
                continue
            elif cmd == "model":
                model = input("Name of model to run: (or 'back' to return to camera selection)\n")
                if cmd == "back":
                    continue
                else:
                    self.camCtrl(stream, True, model)
        rospy.spin()

    def camCtrl(self, id, state, model=None):  # cam id and then true is start and false is stop
        if not id.isnumeric():
            print(f"Invalid ID {id}, please pick from {str(self.camID)}")
            return
        id = int(id)
        if id not in self.camID:
            print(f"Invalid ID of {id}, please pick from {str(self.camID)}")
            return False
        if state and len(self.activeCams) == 2 and id not in self.activeCams:
            print(f"2 camera streams are already active with ids: {str(self.activeCams)}")
            print("Please stop one of these streams first\n")
            return False
        if state and id not in self.activeCams:
            if model == None:
                self.cams[id].start()
            else:
                if id - camAmt < 0:
                    print("Cannot load a model on a nonOak camera")
                    return False
                self.cams[id].start(model)
            self.activeCams.append(id)
            return True
        if not state and id in self.activeCams:
            self.cams[id].kill()
            self.activeCams.remove(id)
            return True
        if state and id in self.activeCams and model != None:
            if id - camAmt < 0:
                print("Cannot load a model on a nonOak camera")
                return False
            self.cams[id].callbackModel(model, True)
            return True

    """
    Essentially it will be a ROS string message formatted as a json.
    The json will include the following information:

    Camera ID: an ID for each camera - Format:
    - If USB low-light: 0 = forward ; 1 = bottom
    - If Oak-D: 10 = forward ; 20 = bottom ; 30 = POE - ^these ids make it significantly easier on my end

    Mode: Start or Stop
    - Format: boolean, True for start and False for stop

    Model (will only be parsed if mode is Start):
    - Format: name of model to run by mission (i.e. “gate”, “buoy”, etc)

    Kill: Only call to kill all streams
    - Format: if a Kill is in the message it will disregard all other params

    Sample Messages:
    Start forward oak with gate model
    {[“camera_ID”]: 10, [“mode”]: True, [“Model”]: “gate”}

    Stop forward oak
    {[“camera_ID”]: 10, [“mode”]: False}
    Start lowlight forward {[“Camera_ID”]: 0, [“mode”]: True}

    Start POE oak without a model
    {[“camera_ID”]: 30, [“mode”]: True, [“Model”]: “raw”} or
    {[“camera_ID”]: 30, [“mode”]: True}

    Kill all active cameras: {[“Kill”]: True}"""

    def callbackCamSelect(self, msg):
        data = json.loads(msg.data)
        print("Received data:", data)
        kill = data.get("kill")
        if kill != None:
            for i in self.activeCams:
                self.camCtrl(i, False)
        camID = data.get("camera_ID")
        mode = data.get("mode")
        model = data.get("model")
        if camID >= 10:
            camID = (camAmt - 1) + camID / 10
        if mode == "start":
            self.camCtrl(str(int(camID)), True, model)
        if mode == "stop":
            self.camCtrl(str(int(camID)), False)

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
    my_node = cameraStreams(True)
    my_node.start()

# https://youtu.be/2l913YwWYe4
# https://stackoverflow.com/questions/55377442/how-to-subscribe-and-publish-images-in-ros

# At the top of pyfakewebcam.py and __init__.py add "from __future__ import absolute_import"
# path: /home/inspiration/.local/lib/python2.7/site-packages/pyfakewebcam
