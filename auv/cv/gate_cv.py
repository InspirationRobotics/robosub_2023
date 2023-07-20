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
        #frame is going to be 640 x 480
        self.step = 0
        self.maxGlyphLength = 0
        self.CENTER_FRAME_X = 320
        logger.info("Gate CV init")

    def run(self, frame, target, detections):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        if target == 0, abydos
        if target == 1, earth
        """
        logger.info("Template CV run")
        #if detections is None or len(detections) == 0:
           # return {"lateral": 0, "forward": 1,"yaw":0, "end": False}, frame
        
        target_x = -1
        #confidenceGate = -1
        targetConfidences = []
        end = False

        for detection in detections:
            targetConfidences.append((detection.confidence, detection.xmin, detection.label))
            print(detection.confidence)

        # Finding which symbol is detected with highest confidence rate
        maxConfidence = 0
        target_label = -1
        for det_confidence, det_x, det_label in targetConfidences:
            # if(confidence[2]==target and confidence[0]>maxTargetConfidence):
            #     maxTargetConfidence = confidence[0]
            #     targetGate = confidence[1]
            if det_label=="E":
                maxConfidence = det_confidence
                target_x = det_x
                target_label = det_label
                
        # Finding center of gate
        print(target_x, self.CENTER_FRAME_X)
        # for i in range(300):
        #     frame[i][target_x] = (255,255,255)
        # #Feedback Loop
        # #strafe until we hit the center of the gate (ensure that we don't lose teh image)
        # #parallel to abydos (yaw until the length of highesgt confidence detection is longest, continue moving until it gets smaller)
        # #strafe until we hit the center of the highest confidence glyph
        forward = 0
        lateral = 0
        yaw = 0
        tolerance = 10
        alignedTarget = -1
        # step 0: strafe until we hit the center of the highest confidence glyph
        if(self.step==0):
            # if target is detected
            if(target_x!=-1):
                if(target_x < self.CENTER_FRAME_X-tolerance):
                    print("strafe left")
                    lateral=-2
                elif(target_x>self.CENTER_FRAME_X+tolerance):
                    print("strafe right")
                    lateral=2
                else:
                    print("aligned, continue")
                    forward=2
                    alignedTarget = target_label
                    self.step=1
        # step 1: keep moving forward until you passed the gate
        if self.step == 1:
            forward = 2
        print(forward, lateral, yaw)
        return {"lateral": lateral, "forward": forward, "yaw":yaw, "end": end, "target": alignedTarget}, frame
        
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
        #return {"lateral": lateral, "forward": forward,"yaw":yaw, "end": end}, frame
