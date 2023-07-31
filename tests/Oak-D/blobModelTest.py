from pathlib import Path
import sys
import os
import cv2
import depthai as dai
import numpy as np
import time

nnBlobPath = str((Path(__file__).parent / Path('dummy3shave.blob')).resolve().absolute())

IMG_H = 480
IMG_W = 640

if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# What are the classes you looking for? Only targets in our case
labelMap = [
            "Class_0",
            "Class_1"
        ]

classAmt = len(labelMap)
confidence = 0.5
anchors = []
anchorMasks = {}

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
camRgb.setPreviewSize(IMG_W, IMG_H)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
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
#detectionNetwork.setNumInferenceThreads(2) needed?
detectionNetwork.input.setBlocking(False)

# Linking
camRgb.preview.link(detectionNetwork.input)
detectionNetwork.passthrough.link(xoutRgb.input)
detectionNetwork.out.link(nnOut.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    
    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
    
    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)
    printOutputLayersOnce = True
    while True:
        inPreview = previewQueue.get()
        inDet = detectionNNQueue.get()
        frame = inPreview.getCvFrame()

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        detections = inDet.detections
        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]
        past_width = 0
        center_x = 0
        center_y = 0
        #Look at the bouding boxes. Find the bigest one - Biggest target.
        for detection in detections:
            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            if((x2-x1)>past_width):
                past_width = x2-x1
                center_x = int((x1 + x2)/2)
                center_y = int((y1+y2)/2)
            
            try:
                label = labelMap[detection.label]
            except:
                label = detection.label
            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"{detection.confidence * 100:.2f}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
           
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

        cv2.putText(frame, f"NN fps: {fps:.2f}", (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        frame = cv2.circle(frame, (center_x, center_y), radius=0, color=(246, 70, 255), thickness=10)
        cv2.imshow("rgb", frame)

        if cv2.waitKey(1) == ord('q'):
            break