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
    model = "gate3"

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
        self.area = 0
        self.flag = [False, False]
        self.config = config
        self.current_sub = self.config.get("sub", "onyx")
        if(self.current_sub == "graey"):
            self.area = 250
        elif(self.current_sub == "onyx"):
            self.area = 310
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
    
    def yawPerpendicular(self, right, left):

        area_ratio = right/left
        # need to tweak tolerances later
        if(area_ratio>1.1):
            yaw = 1
        elif(area_ratio<0.9):
            yaw = -1
        else:
            yaw = 0
        
        return yaw
    
    def outFrameLateral(self, target_x, other_x, tolerance):
        if(target_x < other_x and 0<target_x<tolerance):
            lateral = 1
        elif(other_x < target_x and 0<other_x<tolerance):
            lateral = 1
        elif(target_x < other_x and ((640-tolerance)<other_x<640)):
            lateral = -1
        elif(other_x < target_x and ((640-tolerance)<target_x<640)):
            lateral = -1
        else:
            lateral = 0
        return lateral

    def alignMidpointYaw(self, midpoint, tolerance):
        if midpoint < self.CENTER_FRAME_X - tolerance:
            yaw = -1
        elif midpoint > self.CENTER_FRAME_X + tolerance:
            yaw = 1
        else:
            yaw=0
        
        return yaw
    
    def outFrameYaw(self, target_x, other_x, tolerance):
        if(target_x < other_x and 0<target_x<tolerance):
            yaw=0.65
        elif(other_x < target_x and 0<other_x<tolerance):
            yaw=0.65
        elif(target_x < other_x and ((640-tolerance)<other_x<640)):
            yaw = -0.65
        elif(other_x < target_x and ((640-tolerance)<target_x<640)):
            yaw = -0.65
        else:
            yaw = 0
        return yaw
        
    def edgeFrame(self, target_x, other_x, tolerance):
        left = 0
        right = 0
        if target_x > other_x:
            left, right = other_x, target_x
        else:
            right, left = other_x, target_x

        if(0 < left < tolerance and (640 - tolerance) < right < 640):
            forward = -1
        else:
            forward = 0
        return forward

    def alignTwo(self, detections):
        if(len(detections) == 1):
            self.prevLateral = 2
        elif(len(detections) == 0):
            self.prevLateral = -1*(self.prevLateral)
        elif(len(detections) == 2):
            self.prevLateral = 0
        return self.prevLateral
    
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
            elif(target_area<self.area):
                message = "TOO FAR! GOING FORWARD"
                forward = 1.5
                # if(len(detections) == 2):
                #     midpoint = (target_x + other_x)/2
                #     dist = abs(midpoint - 320) / 320
                #     yaw = self.alignMidpointYaw(midpoint, 20)
                #     yaw = np.clip(yaw * dist * 4, -1, 1)       
            else:
                self.state = "strafe"
                message = "JUST RIGHT! BEGINNING STRAFE/YAW"

            cv2.putText(frame, message, (120, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        # if(self.state == "one"):
        #     latval = self.alignTwo(detections)
        #     if(latval!=0):
        #         lateral = latval
        #     else:
        #         self.state = "strafe"      
        if(self.state == "strafe" and len(detections) == 2):
            midpoint = (target_x + other_x)/2

            dist = abs(midpoint - 320) / 320

            cv2.putText(frame, "ALIGNING STRAFE", (220, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            lateral = self.alignMidpoint(midpoint, 50)
            if(lateral == 0):
                message = "FINISHED STRAFE"
                self.state = "yaw"
                self.flag[0] = True
            latval = self.outFrameLateral(target_x, other_x, 50)
            if(latval!=0 and self.outPrev==False):
                message = "OUT OF FRAME"
                cv2.putText(frame, message, (220, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                self.state = "yaw"
                self.outPrev = True
            if(latval==0 and self.outPrev):
                message = "BACK IN FRAME"
                cv2.putText(frame, message, (220, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                self.outPrev = False

            lateral = np.clip(lateral * dist * 4.5, -1, 1)
            cv2.putText(frame, message, (220, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        elif(self.state == "strafe" and len(detections) == 1):
            self.state = "targetOneDetect"
        elif(self.state=="yaw" and len(detections) == 2):
            message = "ALIGNING YAW"
            if(target_area > other_area):
                r = abs(1-(target_area/other_area))
            else:
                r = abs(1-(other_area/target_area))

            if(target_x > other_x):
                yaw = self.yawPerpendicular(target_area, other_area)
            else:
                yaw = self.yawPerpendicular(other_area, target_area)

            if(yaw==0):
                self.flag[1] = True
                message = "FINISHED YAW"
                self.state ="target"
            yawval = self.outFrameYaw(target_x, other_x, 50)
            if(yawval!=0 and self.outPrev==False):
                message = "OUT OF FRAME"
                cv2.putText(frame, message, (220, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                self.outPrev = True
                self.state = "strafe"
            if(yawval==0 and self.outPrev):
                message = "BACK IN FRAME"
                cv2.putText(frame, message, (220, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                self.outPrev = False
            
            yaw = np.clip(yaw * r * 3, -0.65, 0.65)
            cv2.putText(frame, message, (220, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        if(self.state=="target"):
            message = "ALIGNING WITH TARGET IMAGE"
            lateral = self.alignTarget(target_x, 5)
            if(lateral==0):
                message = "TARGET IS ALIGNED, GOING FORWARD"
                self.state = "end"
        elif(self.state=="targetOneDetect"):
            if len(detections) == 2:
                self.state = "strafe"
                forward=0
            else:
                if(target_x == -1):
                    target_x = other_x
                elif(other_x == -1):
                    target_x = target_x
                message = "ALIGNING WITH TARGET IMAGE"
                lateral = self.alignTarget(target_x, 5)
                forward = 0.6
                if(lateral==0):
                    forward=0
                    message = "TARGET IS ALIGNED, GOING FORWARD"
                    self.state = "end"

            cv2.putText(frame, message, (220, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        if(self.state == "end"):
            end = True
            print("ENDING MISSION")

        if yaw == 0:
            yaw = 0.1

        cv2.putText(frame, "LATERAL:   "+str(lateral)+"\n" +"FORWARD:   "+str(forward) +"\n" + "YAW:   "+str(yaw) , (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame

        # TODO://detection of going through the gate + yaw 2 rotations entirely 