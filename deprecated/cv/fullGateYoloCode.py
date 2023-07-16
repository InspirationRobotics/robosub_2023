import rospy
from sensor_msgs.msg import Image
from pathlib import Path
import sys
import os
import cv2
import depthai as dai
import numpy as np
import time
import serial
import math
from cv_bridge import CvBridge, CvBridgeError
import mavros_msgs.msg
import mavros_msgs.srv

offset = 0      #if the camera is not in the center of the shooter, you can make an offset so that the center of the target is a bit to the right or to the left
middle_width = 19  #The center of the target will not be perfectly aligned with the center of the camera, so this is a wiggle-room. How wide do you want the center to count? (+- 19 pixels in this case)


## Return what needs to be done 

br = CvBridge()
pubForward = rospy.Publisher('/auv/camera/videoUSBOutput0', Image, queue_size=10)
pubThrusters = rospy.Publisher('/auv/devices/thrusters', mavros_msgs.msg.OverrideRCIn, queue_size=10)
#Read the blob file. Copied from the code from Oak D Lite creators
rospy.init_node("CV", anonymous=True)
rospy.Rate(30)


nnBlobPath = str((Path(__file__).parent / Path('smallestmodel_openvino_2022.1_6shave.blob')).resolve().absolute())
if 1 < len(sys.argv):
    arg = sys.argv[1]
    if arg == "yolo3":
        nnBlobPath = str((Path(__file__).parent / Path('../models/yolov5_openvino_2021.4_6shave.blob')).resolve().absolute())
    elif arg == "yolo4":
        nnBlobPath = str((Path(__file__).parent / Path('../models/yolov5_openvino_2021.4_6shave.blob')).resolve().absolute())
    else:
        nnBlobPath = arg
else:
    print("Using YOLOv5 Tin Can Detection")

if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')









# What are the classes you looking for? Only targets in our case
labelMap = ["A","E"]

syncNN = True








# Set up the Oak D Lite Camera

pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
nnNetworkOut = pipeline.create(dai.node.XLinkOut)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutBoundingBoxDepthMapping = pipeline.create(dai.node.XLinkOut)


xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
nnNetworkOut.setStreamName("nnNetwork")


camRgb.setPreviewSize(640,640)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)


stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(1)   
spatialDetectionNetwork.setDepthUpperThreshold(50000)


# Yolo custom specific parameters for tin detection
spatialDetectionNetwork.setNumClasses(2)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors([10,13, 16,30, 33,23, 30,61, 62,45, 59,119, 116,90, 156,198, 373,326])
spatialDetectionNetwork.setAnchorMasks({ "side52": [0,1,2], "side26" : [3,4,5],"side13": [6,7,8] })
spatialDetectionNetwork.setIouThreshold(0.5)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)
spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)


spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    
    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
    
    networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False);

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)
    printOutputLayersOnce = False
    stepOne = False
    maxGlyphLength = 0
    stepTwo = False
    stepThree = False
    while True:
        inPreview = previewQueue.get()
        inDet = detectionNNQueue.get()
        #depth = depthQueue.get()
        inNN = networkQueue.get()
        frame = inPreview.getCvFrame()
        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        detections = inDet.detections
        a = 18*[1500]
        if(len(detections)!=0):
            # If the frame is available, draw bounding boxes on it and show the frame
            height = frame.shape[0]
            width  = frame.shape[1]
            centerOfGate = -1
            abydosGate = -1
            sumOfDets = 0
            abydosConfidences = []
            maxConfidence = 0
            for detection in detections:
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                label = detection.label
                sumOfDets+=detection.xmin*width
                cv2.putText(frame, str(detection.label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
                if(detection.label == 0):
                   cv2.rectangle(frame, (x1, y1), (x2, y2), (0,0,255), cv2.FONT_HERSHEY_SIMPLEX)
                   abydosConfidences.append((detection.confidence,x1))
            for confidence in abydosConfidences:
                if confidence[0]>maxConfidence:
                    maxConfidence = confidence[0]
                    abydosGate = confidence[1]
            if(len(detections)!=0):
                centerOfGate = int(sumOfDets/len(detections))
                for i in range(300):
                    frame[i][centerOfGate] = (255,255,255)
                #Feedback Loop
                #strafe until we hit the center of the gate (ensure that we don't lose teh image)
                #parallel to abydos (yaw until the length of highesgt confidence detection is longest, continue moving until it gets smaller)
                #strafe until we hit the center of the highest confidence glyph
                tolerance = 10
                if(stepOne== False):
                    if(centerOfGate!=-1):
                        if(centerOfGate<320-tolerance):
                            print("strafe right")
                            a[5]=1580
                        elif(centerOfGate>320+tolerance):
                            print("strafe left")
                            a[5]=1420
                        else:
                            print("aligned, continue")
                            stepOne = True 
                if(stepOne):
                    if(abydosGate!=-1):
                        for i in range(300):
                            frame[i][abydosGate] = (0,0,255)
                            if(abydosGate<320-tolerance):
                                print("turn left")
                                a[3]=1450
                            elif(abydosGate>320+tolerance):
                                print("turn right")
                                a[3]=1550
                            else:
                                print("aligned, continue")
                            lengthOfGlyph = x2-x1
                            if(lengthOfGlyph<maxGlyphLength):
                                stepTwo = True
                            else:
                                maxGlyphLength = lengthOfGlyph
                if(stepTwo):
                    if(abydosGate!=-1):
                        if(abydosGate<320-tolerance):
                            print("strafe right")
                            a[5]=1580
                        elif(abydosGate>320+tolerance):
                            print("strafe left")
                            a[5]=1420
                        else:
                            print("aligned, continue")
                            stepThree = True
                if(stepThree):
                    print("go forward")
                    a[4]=1580
                #Absolute Heading
                #hfov = 69
                #headingAngle = 90 - round(math.degrees(math.acos(((abydosGate - width/2) * math.cos(math.radians(90-hfov/2)) / (width/2)))))
                #frame = cv2.putText(frame, str(headingAngle), (abydosGate, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
               # print("Amount needed to turn:" + str(headingAngle))
            #Look at the bouding boxes. Find the bigest one - Biggest target.
        
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = a
        print(a)
        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        pubForward.publish(br.cv2_to_imgmsg(frame))
        #pubThrusters.publish(pwm)

        if cv2.waitKey(1) == ord('q'):
            break
