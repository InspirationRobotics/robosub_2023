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
        self.side = ""
        self.value = ""
        self.CENTER_FRAME_X = 320
        print("[INFO] Gate CV init")

    def smartSide(self, target_x, other_x, other_area, target_area):
        if((target_x < other_x) and (other_area>target_area)):
            self.side = "Right"
            self.value = "Other"
        elif ((other_x < target_x) and (target_area>other_area)):
            self.side = "Right"
            self.value = "Target"
        elif ((other_x < target_x) and (target_area<other_area)):
            self.side = "Left"
            self.value = "Other"
        elif ((other_x > target_x) and (target_area>other_area)):
            self.side = "Left"
            self.value = "Target"
        
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

        other_x = -1
        other_length = -1
        other_area = -1
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
                target_x = det_x
                target_length = det_length
                target_area = det_area
            if det_label == "A":
                other_x = det_x
                other_length = det_length
                other_area = det_area

        forward = 0
        lateral = 0
        yaw = 0
        tolerance = 10
        x=0
        # step 0: strafe until we hit the center of the highest confidence glyph
        # if target is detected

        if self.step == 0:  
            self.smartSide(target_x, other_x, other_area, target_area)
            if target_x < self.CENTER_FRAME_X - tolerance:
                # print("strafe left")
                yaw = -1
            elif target_x > self.CENTER_FRAME_X + tolerance:
                # print("strafe right")
                yaw = 1
            else:
                print("aligned, continue")
                self.step = 1

        # start turning process
        elif self.step == 1:
            print("STARTING step 1")
            
            if(self.value == "Target"):
                x = target_x
            elif(self.value == "Other"):
                x = other_x

            print(self.side, self.value)
            if(self.side == "Right"):
                lateral = -2
                if((self.CENTER_FRAME_X-tolerance)<x<(self.CENTER_FRAME_X+tolerance)):
                    self.step = 2
            
            if(self.side == "Left"):
                if((self.CENTER_FRAME_X-tolerance)<x<(self.CENTER_FRAME_X+tolerance)):
                    self.step = 2
                lateral = 2

        elif self.step == 2:
            print("STARTING step 2")
            if(self.value == "Target"):
                x = target_x
            elif(self.value == "Other"):
                x = other_x

            if(self.side == "Right"):
                yaw = -1
                if((self.CENTER_FRAME_X-tolerance)<x<(self.CENTER_FRAME_X+tolerance)):
                    self.step = 3
            elif(self.side == "Left"):
                if((self.CENTER_FRAME_X-tolerance)<x<(self.CENTER_FRAME_X+tolerance)):
                    self.step = 3
                yaw = 1

        elif self.step == 3:
            print("STARTING step 3")
            forward = 2
            if target_x < self.CENTER_FRAME_X - tolerance:
                # print("strafe left")
                lateral = -2
            elif target_x > self.CENTER_FRAME_X + tolerance:
                # print("strafe right")
                lateral = 2

        

        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame

        # TODO://detection of going through the gate + yaw 2 rotations entirely 