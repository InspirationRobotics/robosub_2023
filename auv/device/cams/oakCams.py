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

from auv.device.cams import pyfakewebcam
from auv.utils import deviceHelper


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
        self.isKilled = True
        self.modelPath = -1
        self.fake = pyfakewebcam.FakeWebcam(newDevice, self.IMG_W, self.IMG_H)
        self.pubFrame = self.rospy.Publisher(f"/auv/camera/videoOAKdRaw{self.name}", Image, queue_size=10)
        self.pubData = self.rospy.Publisher(f"/auv/camera/videoOAKdData{self.name}", String, queue_size=10)
        self.rospy.Subscriber(f"/auv/camera/videoOAKdModel{self.name}", String, self.callbackModel)
        self.rospy.Subscriber(f"/auv/camera/videoOAKdOutput{self.name}", Image, self.callbackMain)
        self.time = time.time()
        print(f"Camera ID {str(id)}: Oak-D {self.name} is available at {newDevice}")

    def createPipeline(self, modelPath=None, confidence=0.5):
        
        self.modelPath = modelPath
        if self.modelPath == None:
            pipeline = dai.Pipeline()
            xoutRgb = pipeline.createXLinkOut()
            xoutRgb.setStreamName("rgb")
            controlIn = pipeline.create(dai.node.XLinkIn)
            controlIn.setStreamName("control")
            camRgb = pipeline.createColorCamera()
            if self.name == "Forward":
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
            else:
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setIspScale(2, 3)
            camRgb.setPreviewSize(self.IMG_W, self.IMG_H)
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
            # setup json parse for .blob file and parameters
            jsonFile = glob.glob(f"{self.modelPath}*.json")[0]
            blobFile = glob.glob(f"{self.modelPath}*.blob")[0]
            print("Found model; creating pipeline")
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
            if self.name == "Forward":
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
            else:
                camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            # camRgb.setIspScale(2, 3)
            # camRgb.setPreviewSize(640, 480)
            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            camRgb.setFps(30)

            # Network specific settings
            detectionNetwork.setConfidenceThreshold(confidence)
            detectionNetwork.setNumClasses(classAmt)
            detectionNetwork.setCoordinateSize(4)
            detectionNetwork.setAnchors(anchors)
            detectionNetwork.setAnchorMasks(anchorMasks)
            detectionNetwork.setIouThreshold(0.5)
            detectionNetwork.setBlobPath(nnBlobPath)
            # detectionNetwork.setNumInferenceThreads(2) needed?
            detectionNetwork.input.setBlocking(False)

            # Linking
            camRgb.preview.link(detectionNetwork.input)
            detectionNetwork.passthrough.link(xoutRgb.input)
            detectionNetwork.out.link(nnOut.input)

            # Initializing Oak-D with pipeline
            device_info = dai.DeviceInfo(self.mxid)
            self.device = dai.Device(pipeline, device_info)

        self.isKilled = False

    def mxidToName(self, mxid):
        if mxid == deviceHelper.dataFromConfig("forwardOak"):
            return "Forward"
        elif mxid == deviceHelper.dataFromConfig("bottomOak"):
            return "Bottom"
        elif mxid == deviceHelper.dataFromConfig("poeOak"):
            return "Poe"
        else:
            return "Unknown"

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

    def modelSelect(self, modelName):
        modelsList = ["gate", "dhd", "gateAug", "buoy", "raw"]
        if modelName not in modelsList:
            if modelName[0] == "/":
                print("Detected direct path")
                if os.path.exists(modelName):
                    print(f"Switching {self.name} oakD to model at: {modelName}")
                    if not modelName.endswith("/"):
                        modelName += "/"
                    modelPath = modelName
                else:
                    print(f"Path not detected ({self.name} oakD error")
                    return
            else:
                print(f"Model {modelName} not found ({self.name} oakD error)")
                return
        elif modelName == "raw":
            print(f"Switching {self.name} oakD to raw view")
            modelPath = None
        else:
            print(f"Switching {self.name} oakD to {modelName} model")
            folderPath = "/home/inspiration/auv/auv/device/cams/models/"
            modelPath = f"{folderPath + modelName}Model/"
        if self.modelPath == modelPath:
            print("Model already running...")
            return
        return modelPath

    def callbackModel(self, msg, debug=False):
        if self.isKilled:
            return
        if debug:
            modelName = msg
        else:
            modelName = msg.data
        if(self.modelSelect(modelName)==None):
            return
        self.kill()
        self.start(modelName)

    def runner(self):
        cam = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        if self.modelPath != None:
            qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
        while not self.rospy.is_shutdown() and not self.isKilled:
            try:
                frame1 = cam.get().getCvFrame()
                if self.modelPath != None:
                    inDet = qDet.get()
                    detections = inDet.detections
                    dataToSend = {}
                    for i, detection in enumerate(detections):
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
                        cv2.putText(
                            frame1,
                            f"{detection.confidence * 100:.2f}",
                            (x1 + 10, y1 + 35),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255,
                        )
                        cv2.rectangle(frame1, (x1, y1), (x2, y2), (255, 255, 255), cv2.FONT_HERSHEY_SIMPLEX)  # change for multiple colors?
                        dataToSend[str(i)] = [label, round(detection.confidence * 100, 3), x1, x2, y1, y2]
                    self.pubData.publish(str(json.dumps(dataToSend)))
                msg = self.br.cv2_to_imgmsg(frame1)
                self.pubFrame.publish(msg)
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
        self.isKilled = True
        self.rospy.loginfo(f"Killing Camera {str(self.id)} Stream...")
        self.oakThread.join()
        del self.device
        self.rospy.loginfo(f"Killed Camera {str(self.id)} Stream...")
        pass  # todo

    def start(self, modelPath="raw"):
        if not self.isKilled:
            return
        self.createPipeline(self.modelSelect(modelPath))
        self.isKilled = False
        self.rospy.loginfo(f"Starting Camera {str(self.id)} Stream...")
        self.oakThread = threading.Timer(0, self.runner)
        self.oakThread.daemon = True
        self.oakThread.start()
