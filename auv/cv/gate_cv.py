"""
Description: CV template class
Author: Team Inspiration
"""

# import what you need from within the package

import logging
import time

import cv2
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class CV:
    """Template CV class, don't change the name of the class"""

    camera = "/auv/camera/videoOAKdRawForward"

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """
        
        self.step = 0
        self.maxGlyphLength = 0
        self.CENTER_FRAME_X = 320
        logger.info("Template CV init")

    def run(self, frame, target, detections):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        if target == 0, abydos
        if target == 1, earth
        """
        print(frame)
        logger.info("Template CV run")

        if detections is None or len(detections) == 0:
            return {"lateral": 0, "forward": 0,"yaw":0, "end": False}, frame
        
        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]
        target_x = -1
        #confidenceGate = -1
        sumOfDets = 0
        targetConfidences = []
        end = False

        for detection in detections:
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            sumOfDets+=detection.xmin*width
            cv2.putText(frame, str(detection.label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), cv2.FONT_HERSHEY_SIMPLEX)
            targetConfidences.append((detection.confidence, x1, detection.label))

        # Finding which symbol is detected with highest confidence rate
        maxConfidence = 0
        target_label = -1
        for det_confidence, det_x, det_label in targetConfidences:
            # if(confidence[2]==target and confidence[0]>maxTargetConfidence):
            #     maxTargetConfidence = confidence[0]
            #     targetGate = confidence[1]
            if det_confidence>maxConfidence:
                maxConfidence = det_confidence
                target_x = det_x
                target_label = det_label
                
        # Finding center of gate
        
        for i in range(300):
            frame[i][target_x] = (255,255,255)
        #Feedback Loop
        #strafe until we hit the center of the gate (ensure that we don't lose teh image)
        #parallel to abydos (yaw until the length of highesgt confidence detection is longest, continue moving until it gets smaller)
        #strafe until we hit the center of the highest confidence glyph
        tolerance = 10
        alignedTarget = -1
        # step 0: strafe until we hit the center of the highest confidence glyph
        if(self.step==0):
            # if target is detected
            if(target_x!=-1):
                if(target_x < self.CENTER_FRAME_X-tolerance):
                    print("strafe left")
                    lateral=-1
                elif(target_x>self.CENTER_FRAME_X+tolerance):
                    print("strafe right")
                    lateral=1
                else:
                    print("aligned, continue")
                    self.step = 1
                    alignedTarget = target_label
            else:
                yaw = 0.5
        # step 1: keep moving forward until you passed the gate
        elif self.step == 1:
            forward = 1
        return {"lateral": lateral, "forward": forward,"yaw":yaw, "end": end, "target": alignedTarget}, frame
        
        # TODO://detection of going through the gate + yaw 2 rotations entirely

        '''
        elif(self.step == 1):
            if(confidenceGate!=-1):
                for i in range(300)
                    frame[i][confidenceGate] = (0,0,255)
                    if(confidenceGate<320-tolerance):
                        print("turn left")
                        yaw=-1
                    elif(confidenceGate>320+tolerance):
                        print("turn right")
                        yaw=1
                    else:
                        print("aligned, continue")
                    lengthOfGlyph = x2-x1
                    if(lengthOfGlyph<maxGlyphLength):
                        self.step = 2
                    else:
                        maxGlyphLength = lengthOfGlyph
        # step 2: strafe to align with confidence image
        elif(self.step == 2):
            if(confidenceGate!=-1):
                if(confidenceGate<320-tolerance):
                    print("strafe right")
                    lateral=1
                elif(confidenceGate>320+tolerance):
                    print("strafe left")
                    lateral=-1
                else:
                    print("aligned, continue")
                    self.step = 3
        # step 3: strafe to align with final target image
        elif(self.step == 3):
            if(targetGate!=-1):
                if(targetGate<320-tolerance):
                    print("strafe right")
                    lateral=1
                elif(targetGate>320+tolerance):
                    print("strafe left")
                    lateral=-1
                else:
                    print("aligned, continue")
                    self.step = 4
        # step 3: go forward and pass through gate
        elif(self.step == 4):
            print("go forward")
            forward=1
            # yaw for 2 full rotations
            end = True
        '''
        return {"lateral": lateral, "forward": forward,"yaw":yaw, "end": end}, frame
