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
        self.ratio = 1
        self.state = "strafe"
        self.flag = [False, False]
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
    
    def alignMidpoint(self, midpoint, tolerance):
        if midpoint < self.CENTER_FRAME_X - tolerance:
            lateral = -1
        elif midpoint > self.CENTER_FRAME_X + tolerance:
            lateral = 1
        else:
            lateral=0
        
        return lateral

    def alignTarget(self, target_x, tolerance):
        if target_x < self.CENTER_FRAME_X - tolerance:
            lateral = -1
        elif target_x > self.CENTER_FRAME_X + tolerance:
            lateral = 1
        else:
            lateral = 0
        
        return lateral
    
    def yawPerpendicular(self, target_area, other_area):
        area_ratio = target_area/other_area
        # need to tweak tolerances later
        if(area_ratio>1.2):
            yaw = -0.5
        elif(area_ratio<0.8):
            yaw = 0.5
        else:
            yaw = 0
        
        return yaw
    
    def outFrameLateral(self, target_x, other_x, tolerance):
        if(target_x < other_x and 0<target_x<tolerance):
            lateral=2
        elif(other_x < target_x and 0<other_x<tolerance):
            lateral=2
        elif(target_x < other_x and ((640-tolerance)<other_x<640)):
            lateral = -2
        elif(other_x < target_x and ((640-tolerance)<target_x<640)):
            lateral = -2
        else:
            lateral = 0
        return lateral

    def outFrameYaw(self, target_x, other_x, tolerance):
        if(target_x < other_x and 0<target_x<tolerance):
            yaw=1
        elif(other_x < target_x and 0<other_x<tolerance):
            yaw=1
        elif(target_x < other_x and ((640-tolerance)<other_x<640)):
            yaw = -1
        elif(other_x < target_x and ((640-tolerance)<target_x<640)):
            yaw = -1
        else:
            yaw = 0
        return yaw
        
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
        target_length = -1
        # confidenceGate = -1
        targetConfidences = []
        end = False
        print(self.state)
        print(self.flag)
        for detection in detections:
            area = abs(detection.xmin - detection.xmax)*abs(detection.ymin-detection.ymax)
            targetConfidences.append((detection.confidence, detection.xmin, detection.label, abs(detection.xmin - detection.xmax), area))

        # Finding which symbol is detected with highest confidence rate
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
        tolerance =20
        x=0
        midpoint = (target_x+other_x)/2
        # step 0: strafe until we hit the center of the highest confidence glyph
        # if target is detected
    
        if(self.state == "strafe"):
            print("ALIGNING STRAFE")
            lateral = self.alignMidpoint(midpoint, tolerance)
            if(lateral==0):
                print("FINISHED STRAFE")
                self.state = "yaw"
                self.flag[0] = True
            latval = self.outFrameLateral(target_x, other_x, 50)
            if(latval!=0):
                print("OUT OF FRAME STRAFE")
                lateral = latval

        if(self.state=="yaw"):
            print("ALIGNING YAW")
            yaw = self.yawPerpendicular(target_area, other_area)
            if(yaw==0):
                self.flag[1] = True
                print("FINISHED YAW")
                if(self.flag[0]):
                    self.state ="target"
            yawval = self.outFrameYaw(target_x, other_x, 50)
            if(yawval!=0):
                print("OUT OF FRAME YAW")
                yaw = yawval

        if(self.state=="target"):
            print("ALIGNING TARGET")
            lateral = self.alignTarget(target_x, tolerance)
            if(lateral==0):
                forward=2

        if(len(detections)==0 and self.state == "target"):
            forward =2
            lateral=0
            yaw=0
        if yaw == 0:
            yaw = 0.1
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame

        # TODO://detection of going through the gate + yaw 2 rotations entirely 