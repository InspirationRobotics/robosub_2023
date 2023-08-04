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
        self.outPrev = False
        self.prevLateral = 0
        self.state = "frame"
        self.flag = [False, False]
        self.CENTER_FRAME_X = 320
        print("[INFO] Gate CV init")

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
        target_ratio = 0
        other_ratio = 0
        # confidenceGate = -1
        targetConfidences = []
        end = False
        for detection in detections:
            area = abs(detection.xmin - detection.xmax)*abs(detection.ymin-detection.ymax)
            x = (detection.xmin + detection.xmax)/2
            ratio = abs(detection.xmin - detection.xmax)/abs(detection.ymin-detection.ymax)
            targetConfidences.append((detection.confidence, x, detection.label, abs(detection.xmin - detection.xmax), area, ratio))

        for det_confidence, det_x, det_label, det_length, det_area, det_ratio in targetConfidences:
            if target in det_label:
                target_x = det_x
                target_length = det_length
                target_area = det_area
                target_ratio = det_ratio

            else:
                other_x = det_x
                other_length = det_length
                other_area = det_area
                other_ratio = det_ratio

        forward = 0
        lateral = 0
        yaw = 0
        tolerance =20
        message = ""
        # step 0: strafe until we hit the center of the highest confidence glyph
        # if target is detected
        if(self.state == "frame"):
            print("AREA:", target_area)
            if(target_area > 650):
                message = "TOO CLOSE! GOING BACKWARD"
                forward = -1
            elif(target_area<310):
                message = "TOO FAR! GOING FORWARD"
                forward = 1
            else:
                self.state = "strafe"
                message = "JUST RIGHT! BEGINNING STRAFE/YAW"

            cv2.putText(frame, message, (120, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
  
        if(self.state == "strafe" and len(detections) == 2):
            midpoint = (target_x + other_x)/2

            dist = abs(midpoint - 320) / 320

            cv2.putText(frame, "ALIGNING STRAFE", (220, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            lateral = self.alignMidpoint(midpoint, 20)
            if(lateral == 0):
                message = "FINISHED STRAFE"
                self.state = "end"
            
            lateral = np.clip(lateral * dist * 5.5, -1, 1)
            cv2.putText(frame, message, (220, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
        if(self.state == "end"):
            end = True
            print("ENDING MISSION")

        if yaw == 0:
            yaw = 0.1

        cv2.putText(frame, "LATERAL:   "+str(lateral)+"\n" +"FORWARD:   "+str(forward) +"\n" + "YAW:   "+str(yaw) , (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame

        # TODO://detection of going through the gate + yaw 2 rotations entirely 