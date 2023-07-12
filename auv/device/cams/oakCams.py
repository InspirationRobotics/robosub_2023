import os
import platform
import signal
import sys
import threading
import time

import cv2
import json
import glob
import numpy as np
import depthai as dai
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from . import pyfakewebcam

class oakCamera:
    def __init__(self, rospy, id, mxid, newDevice):
        self.IMG_W = 640
        self.IMG_H = 480
        self.rospy = rospy
        self.id = id
        self.mxid = mxid
        self.name = self.mxidToName(mxid)
        self.frame = None
        self.br = CvBridge()
        self.loop_rate = self.rospy.Rate(30)
        self.createPipeline()
        self.fake = pyfakewebcam.FakeWebcam(newDevice, self.IMG_W, self.IMG_H)
        self.pubFrame = self.rospy.Publisher("/auv/camera/videoOAKdRaw" + self.name, Image, queue_size=10)
        self.pubData = self.rospy.Publisher("/auv/camera/videoOAKdData" + self.name, String, queue_size=10)
        self.rospy.Subscriber("/auv/camera/videoOAKd" + self.name + "Model", String, self.callbackModel)
        self.rospy.Subscriber("/auv/camera/videoOAKdOutput" + self.name, Image, self.callbackMain)
        self.time = time.time()
        print("Oak-D " + self.name + " is available at " + newDevice)

    def createPipeline(self, modelPath=None, confidence=0.5):
        self.initialized = False
        self.modelPath = modelPath
        if(self.modelPath==None):
            pipeline = dai.Pipeline()
            xoutRgb = pipeline.createXLinkOut()
            xoutRgb.setStreamName("rgb")
            controlIn = pipeline.create(dai.node.XLinkIn)
            controlIn.setStreamName("control")
            camRgb = pipeline.createColorCamera()
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setIspScale(2, 3)
            camRgb.setPreviewSize(640, 480)
            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            camRgb.preview.link(xoutRgb.input)
            controlIn.out.link(camRgb.inputControl)
            device_info = dai.DeviceInfo(self.mxid)
            self.device = dai.Device(pipeline, device_info)
            controlQueue = self.device.getInputQueue("control")
            # Autofocus trigger (and disable continuous)
            ctrl = dai.CameraControl()
            ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
            ctrl.setAutoFocusTrigger()
            controlQueue.send(ctrl)
        else:
            #setup json parse for .blob file and parameters
            jsonFile = glob.glob(modelPath+"*.json")[0]
            blobFile = glob.glob(modelPath+"*.blob")[0]
            jsonFile = open(jsonFile)
            data = json.load(jsonFile)
            NN_params = data["nn_config"]["NN_specific_metadata"]
            nnBlobPath = blobFile
            classAmt = NN_params["classes"]
            anchors = NN_params["anchors"]
            anchorMasks = NN_params["anchor_masks"]
            self.labelMap = data["mappings"]["labels"]
            
            # Create pipeline
            pipeline = dai.Pipeline()

            # Define sources and outputs
            camRgb = pipeline.create(dai.node.ColorCamera)
            detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
            xoutRgb = pipeline.create(dai.node.XLinkOut)
            nnOut = pipeline.create(dai.node.XLinkOut)

            xoutRgb.setStreamName("rgb")
            nnOut.setStreamName("nn")

            # Properties
            camRgb.setPreviewSize(self.IMG_W, self.IMG_H)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            camRgb.setFps(40)

            # Network specific settings
            detectionNetwork.setConfidenceThreshold(confidence)
            detectionNetwork.setNumClasses(classAmt)
            detectionNetwork.setCoordinateSize(4)
            detectionNetwork.setAnchors(anchors)
            detectionNetwork.setAnchorMasks(anchorMasks)
            detectionNetwork.setIouThreshold(0.5)
            detectionNetwork.setBlobPath(nnBlobPath)
            #detectionNetwork.setNumInferenceThreads(2) needed?
            detectionNetwork.input.setBlocking(False)

            # Linking
            camRgb.preview.link(detectionNetwork.input)
            detectionNetwork.passthrough.link(xoutRgb.input)
            detectionNetwork.out.link(nnOut.input)

            #Initializing Oak-D with pipeline
            device_info = dai.DeviceInfo(self.mxid)
            self.device = dai.Device(pipeline, device_info)

        self.initialized = True

    def mxidToName(self, mxid):
        if mxid == "18443010B1A0B01200":
            return "Forward"
        elif mxid == "184430104161721200":
            return "Bottom"
        elif mxid == "18443010D11AF50F00":
            return "Poe"
        else:
            return "Unknown"

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

    def callbackModel(self, msg):
        pass  # need to implment blob file switching

    def runner(self):
        cam = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        while not self.rospy.is_shutdown():
            if(self.modelPath!=None):
                if(not self.initialized):
                    continue
                qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
            while self.initialized:
                try:
                    frame1 = cam.get().getCvFrame()
                    if(self.modelPath!=None):
                        inDet = qDet.get()
                        detections = inDet.detections
                        for detection in detections:
                            # Denormalize bounding box
                            x1 = int(detection.xmin * self.IMG_W)
                            x2 = int(detection.xmax * self.IMG_W)
                            y1 = int(detection.ymin * self.IMG_H)
                            y2 = int(detection.ymax * self.IMG_H)
                            try:
                                label = self.labelMap[detection.label]
                            except:
                                label = detection.label
                            cv2.putText(frame1, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                            cv2.putText(frame1, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                            cv2.rectangle(frame1, (x1, y1), (x2, y2), (255, 255, 255), cv2.FONT_HERSHEY_SIMPLEX) #change for multiple colors?
                        self.pubData.publish(str(detections))
                    msg = self.br.cv2_to_imgmsg(frame1)
                    self.pubFrame.publish(msg)
                    if(time.time()-self.time>3): #no new CV output frames recieved, default to cam view
                        self.sendFakeFrame(frame1)
                    pass
                except Exception as e:
                    print("Camera " + str(self.id) + " Input Error")
                    print(e)
                self.loop_rate.sleep()

    def kill(self):
        pass #todo

    def start(self):
        self.rospy.loginfo("Starting Camera " + str(self.id) + " Stream...")
        self.thread_param_updater = threading.Timer(0, self.runner)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()