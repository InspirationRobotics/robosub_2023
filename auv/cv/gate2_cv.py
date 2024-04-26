"""
CV script for the gate mission. The difference between gate2_cv.py (this file) and gate_cv.py is that the ML model used to find the detections in this file is
the blob file named "gate", while in gate_cv.py the ML model is "gate3".

Align the sub to enter the correct side of the gate, either Earth or Abydos. This does NOT move the sub through the gate, this is for the style mission.

This code is incomplete; it only aligns the sub with the center of the gate (meaning in between the two sides), not on one of the sides.
"""

import time

import cv2
import numpy as np


class CV:
    """
    CV class for running the Gate mission. DO NOT change the name of the class; doing so will mess up all of the backend files.
    """

    camera = "/auv/camera/videoOAKdRawForward" # Camera to obtain camera feed from
    model = "gate" # ML model to run

    def __init__(self, **config):
        """
        Initialize the CV script

        Args:
            config: Mission-specific parameters to run the gate mission
        """
        # The frame is going to be 640 (width) x 480 (height)
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

    """
    Note that all of these coordinates/values are relative to one another with respect to the size of the camera stream frame
    """
    """
    Lateral: Negative is left, positive is right
    Yaw: Negative is counterclockwise, positive is clockwise
    Forward: Negative is backward, positive is forward

    0 is no movement for all three planes of motion
    """

    def alignMidpoint(self, midpoint, tolerance):
        """
        Align with the midpoint of the gate (in between both sides of the gate).

        Args:
            midpoint: The midpoint of the gate
            tolerance: Error threshold/tolerance
        """
        if midpoint < self.CENTER_FRAME_X - tolerance:
            lateral = -1
        elif midpoint > self.CENTER_FRAME_X + tolerance:
            lateral = 1
        else:
            lateral=0
        
        return lateral

    def alignTarget(self, target_x, tolerance):
        """
        Align with the target side of the gate; this is the center of the target side.

        Args:
            target_x: Target side's center (x coordinate)
            tolerance: Error threshold/tolerance
        """
        if target_x < self.CENTER_FRAME_X - tolerance:
            lateral = -1
        elif target_x > self.CENTER_FRAME_X + tolerance:
            lateral = 1
        else:
            lateral = 0
        
        return lateral

    def run(self, frame, target, detections):
        """
        Run the Gate CV script.

        Args:
            frame: Current frame from the camera stream.
            target (int): Target side (if target == 0, Abydos) (if target == 1, Earth)
            detections: List of detections outputted from the ML model 

        Returns:
            dictionary, frame: {lateral motion command, forward motion command, yaw motion command, flag indicating whether the mission has ended or not}, visualized frame
        """
        #print("[INFO] Gate CV run")
        # if detections is None or len(detections) == 0:
        # return {"lateral": 0, "forward": 1,"yaw":0, "end": False}, frame

        # Initialize variables
        target_x = -1
        target_area = -1
        other_x = -1
        other_length = -1
        other_area = -1
        target_length = -1
        target_ratio = 0
        other_ratio = 0
        # confidenceGate = -1
        targetConfidences = [] # Holds the calculated data for each detection
        end = False

        # Loop through each detection, and calculate the area, center x-coordinate, and the x/y ratio. Append the all of this to targetConfidences
        for detection in detections:
            area = abs(detection.xmin - detection.xmax)*abs(detection.ymin-detection.ymax)
            x = (detection.xmin + detection.xmax)/2
            ratio = abs(detection.xmin - detection.xmax)/abs(detection.ymin-detection.ymax)
            targetConfidences.append((detection.confidence, x, detection.label, abs(detection.xmin - detection.xmax), area, ratio))

        # If, based on the detection label, the detection is the target side, then set the target data to that detection's data
        # Else, the other detection will be the target, so just set the other side's data to that detection's data
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

        # If the target is detected, move forward and backwards so that the target side's area as seen on the frame is the right size.
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

            cv2.putText(frame, message, (120, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2) # Put the current frame with the message on the screen
  
        # Strafe (move while keeping a forward-facing orientation) and yaw to align with the midpoint of the gate (in between the two sides).
        if(self.state == "strafe" and len(detections) == 2):
            midpoint = (target_x + other_x)/2 # Find the center of the gate

            dist = abs(midpoint - 320) / 320 # Normalize the alignment error between where the midpoint is and where it should be on the frame

            cv2.putText(frame, "ALIGNING STRAFE", (220, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            lateral = self.alignMidpoint(midpoint, 20) # Align the sub with the midpoint of the gate
            if(lateral == 0):
                message = "FINISHED STRAFE"
                self.state = "end"
            
            lateral = np.clip(lateral * dist * 5.5, -1, 1) # Apply a gain then clip
            cv2.putText(frame, message, (220, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
        if(self.state == "end"):
            end = True
            print("ENDING MISSION")

        if yaw == 0:
            yaw = 0.1

        cv2.putText(frame, "LATERAL:   "+str(lateral)+"\n" +"FORWARD:   "+str(forward) +"\n" + "YAW:   "+str(yaw) , (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame

        # TODO://detection of going through the gate + yaw 2 rotations entirely 