"""
Description: CV template class
Author: Team Inspiration
"""

# import what you need from within the package


import time

import cv2
import numpy as np


class CV:
    """Template CV class, don't change the name of the class"""

    camera = "/auv/camera/videoOAKdRawForward"
    model = "gate"

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """
        # frame is going to be 640 x 480
        self.step = 0
        self.prevTargetLength = 0
        self.prevYaw = None
        self.CENTER_FRAME_X = 320
        self.ratio = 1
        self.yaw = 0
        print("[INFO] Gate CV init")

    def run(self, frame, target, detections):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        if target == 0, abydos
        if target == 1, earth
        """
        #print("[INFO] Gate CV run")
        # if detections is None or len(detections) == 0:
        # return {"lateral": 0, "forward": 1,"yaw":0, "end": False}, frame

        target_x = -1
        target_area = -1
        # confidenceGate = -1
        targetConfidences = []
        end = False

        for detection in detections:
            area = abs(detection.xmin - detection.xmax)*abs(detection.ymin-detection.ymax)
            targetConfidences.append((detection.confidence, detection.xmin, detection.label, abs(detection.xmin - detection.xmax), area))

        # Finding which symbol is detected with highest confidence rate
        target_length = -1
        for det_confidence, det_x, det_label, det_length, det_area in targetConfidences:
            if det_label == "E":
                maxConfidence = det_confidence
                target_x = det_x
                target_length = det_length
                target_label = det_label
                target_area = det_area
                print(det_area)

        forward = 0
        lateral = 0
        yaw = 0
        tolerance = 20
        # step 0: strafe until we hit the center of the highest confidence glyph
        # if target is detected
        if self.step == 0:
            yaw = -1
            if(len(detections)!= 0):
                self.step = 1

        if self.step == 1:  
            if target_x < self.CENTER_FRAME_X - tolerance:
                # print("strafe left")
                lateral = -2
            elif target_x > self.CENTER_FRAME_X + tolerance:
                # print("strafe right")
                lateral = 2
            else:
                print("aligned, continue")
                self.step = 2

        # start turning process
        elif self.step == 2:
            if(len(detections)>0):
                if(self.prevYaw == None):
                    yaw = 1
                else:
                    if(self.prevTargetLength<target_length):
                        yaw = self.prevYaw
                    elif (self.prevTargetLength>target_length):
                        yaw = -1*(self.prevYaw)
                        
                forward = 2
                if target_x < self.CENTER_FRAME_X - tolerance:
                    # print("strafe left")
                    lateral = -1
                elif target_x > self.CENTER_FRAME_X + tolerance:
                    # print("strafe right")
                    lateral = 1

            if(target_area > 2200):
                forward=2
                end = True
        self.prevTargetLength = target_length
        self.prevYaw = yaw

        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame

        # TODO://detection of going through the gate + yaw 2 rotations entirely