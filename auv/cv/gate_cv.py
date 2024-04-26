"""
CV script for the gate mission.
This code aligns the sub to the correct side of the gate. It DOES NOT move the sub through the gate -- that is for the style mission.
"""

import time

import cv2
import numpy as np


class CV:
    """
    CV class for running the Gate mission. DO NOT change the name of the class; doing so will mess up all of the backend files
    """
    camera = "/auv/camera/videoOAKdRawForward" # Camera to use
    model = "gate3" # ML model to run

    def __init__(self, **config):
        """
        Initialize the CV script

        Args:
            config: Mission-specific parameters to run the gate mission
        """
        # Frame will be 640 (width) x 480 (height)
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

    """
    Note that all of these coordinates/values are relative to one another with respect to the size of the camera stream frame
    """
    """
    Lateral: Negative is left, positive is right
    Yaw: Negative is counterclockwise, positive is clockwise
    Forward: Negative is backward, positive is forward

    0 is no movement for all three planes of motion
    """

    def smartSide(self, target_x, other_x, other_area, target_area):
        """
        NOTE: This function is not called anywhere at present.
        Determines whether we are to the left or right of the target based on the areas of the different gates that 
        are taken up on the camera stream.

        Args:
            target_x: X-coordinate of the target
            other_x: X-coordinate of the other (side)
            other_area: Other side's area
            target_area: Target's area
        """
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
        """
        Aligns the sub with the midpoint (center x-coordinate) of the target.

        Args:
            midpoint (float/int): The target's midpoint
            tolerance (float/int): The error tolerance

        Returns:
            int: Lateral movement command (-1, 0, 1)
        """
        # Smallest x-coordinates are on the left side, largest x-coordinates are on the right side
        if midpoint < self.CENTER_FRAME_X - tolerance:
            lateral = -1
        elif midpoint > self.CENTER_FRAME_X + tolerance:
            lateral = 1
        else:
            lateral=0
        
        return lateral

    def alignTarget(self, target_x, tolerance):
        """
        NOTE: This does essentially the same thing as AlignMidpoint()
        Aligns the sub with the x-coordinate of the target.

        Args:
            target_x (float/int): The target's x-coordinate
            tolerance (float/int): The error tolerance

        Returns:
            int: Lateral movement command (-1, 0, 1)
        """
        if target_x < self.CENTER_FRAME_X - tolerance:
            lateral = -1
        elif target_x > self.CENTER_FRAME_X + tolerance:
            lateral = 1
        else:
            lateral = 0
        
        return lateral
    
    def yawPerpendicular(self, right, left):
        """
        Yaw clockwise or counterclockwise based on the ratios of the areas to the right and left (move to in between 
        the two sides of the gate -- Earth and Abydos)

        Args:
            right: Area on the right side
            left: Area on the left side
        
        Returns:
            int: Yaw value (either -1, 0, 1)
        """
        area_ratio = right/left
        # Need to tweak tolerances later
        # 1 is clockwise, -1 is counterclockwise
        if(area_ratio>1.1):
            yaw = 1
        elif(area_ratio<0.9):
            yaw = -1
        else:
            yaw = 0
        
        return yaw
    
    def outFrameLateral(self, target_x, other_x, tolerance):
        """
        Determines the lateral movement when objects are at the edge of the frame

        Args:
            target_x (int/float): X-coordinate of the target
            other_x (int/float): X-coordinate of the other gate
            tolerance (int/float): How far objects can be from the edge of the frame

        Returns:
            int: Lateral movement command (-1, 0, 1)
        """
        # 1 is moving to the right, -1 is moving to the left
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
        """
        Yaw to align the sub with the midpoint of the target 

        Args:
            midpoint (int/float): X-coordinate of the target
            tolerance: Error tolerance threshold

        Returns:
            int: Yaw command (-1, 0, 1)
        """
        if midpoint < self.CENTER_FRAME_X - tolerance:
            yaw = -1
        elif midpoint > self.CENTER_FRAME_X + tolerance:
            yaw = 1
        else:
            yaw=0
        
        return yaw
    
    def outFrameYaw(self, target_x, other_x, tolerance):
        """
        Yaw based on whether objects are on the edge of the frame

        Args:
            target_x: Target's x-coordinate
            other_x: Other side's x-coordinate
            tolerance: How close the object can be from the edge of the frame
        """
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
        """
        Determines forward movement when objects are at the edge of the frame. Basically if the objects are on the edge move backwards.

        Args:
            target_x (int/float): Target x-coordinate
            other_x (int/float): Other side's x-coordinate
            tolerance: How close objects can be to the edge of the frame

        Returns:
            int: Forward motion command (0 or -1)
        """
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
        """
        Determines lateral movement based on the number of detections detected.
        NOTE: This function is not called

        Args:
            detections (list): List of detections

        Returns:
            int: Lateral movement command (-2, 0, 2)
        """
        if(len(detections) == 1):
            self.prevLateral = 2
        elif(len(detections) == 0):
            self.prevLateral = -1*(self.prevLateral)
        elif(len(detections) == 2):
            self.prevLateral = 0
        return self.prevLateral
    
    def run(self, frame, target, detections):

        """
        Run the gate CV script. This function will be called continuously by the gate mission file, so there is no need to loop here.

        Args:
            frame: Frame from the camera feed
            target: Target side of gate (if target == 0 -> Abydos | if target == 1 -> Earth)
            detections: The list of detections

        Returns:
            dictionary: {lateral motion command, forward motion command, yaw motion command}, visualized frame
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
        targetConfidences = []
        end = False

        # Find the area of each detection, find the ratio (width/height) of each detection, store the detection with its relevant data
        for detection in detections:
            area = abs(detection.xmin - detection.xmax)*abs(detection.ymin-detection.ymax)
            x = (detection.xmin + detection.xmax)/2
            ratio = abs(detection.xmin - detection.xmax)/abs(detection.ymin-detection.ymax)
            targetConfidences.append((detection.confidence, x, detection.label, abs(detection.xmin - detection.xmax), area, ratio))

        # Find the detection that is the target, and set the corresponding variables in that detection list to be the target's variables
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

        # Step 0: move forward and backward enough so that the target's area on the screen is not larger than the screen, but not smaller than
        # the area of the sub
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

            cv2.putText(frame, message, (120, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2) # Put the current frame with the message on the screen

        # if(self.state == "one"):
        #     latval = self.alignTwo(detections)
        #     if(latval!=0):
        #         lateral = latval
        #     else:
        #         self.state = "strafe"     

        # Step 1:  Strafe (move laterally while keeping a forward-facing orientation) so that the target is in our frame
        if(self.state == "strafe" and len(detections) == 2):
            midpoint = (target_x + other_x)/2

            dist = abs(midpoint - 320) / 320

            cv2.putText(frame, "ALIGNING STRAFE", (220, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            lateral = self.alignMidpoint(midpoint, 50)

            # If there is no other lateral movement necessary
            if(lateral == 0):
                message = "FINISHED STRAFE"
                self.state = "yaw"
                self.flag[0] = True
            latval = self.outFrameLateral(target_x, other_x, 50)
            
            # If there is movement necessary to move the target back into the screen and the target was not out of the frame previously
            if(latval!=0 and self.outPrev==False):
                message = "OUT OF FRAME"
                cv2.putText(frame, message, (220, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                self.state = "yaw"
                self.outPrev = True
            
            # If the target is back in the frame (no more movement necessary to move the target back into the frame)
            if(latval==0 and self.outPrev):
                message = "BACK IN FRAME"
                cv2.putText(frame, message, (220, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                self.outPrev = False

            lateral = np.clip(lateral * dist * 4.5, -1, 1)
            cv2.putText(frame, message, (220, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        elif(self.state == "strafe" and len(detections) == 1):
            self.state = "targetOneDetect"

        # Step 2: Yaw so that we are dead-center in between the two sides (Earth and Abydos)
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

            # If the sub doesn't have to yaw anymore
            if(yaw==0):
                self.flag[1] = True
                message = "FINISHED YAW"
                self.state ="target"

            yawval = self.outFrameYaw(target_x, other_x, 50)
            
            # Handle yawing so that both sides of the gate are within the frame
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

        # Step 3: Align with the target (after this is completed, that is when the style mission will take over)
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
        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame # Return the motion values and the visualized frame

        # TODO://detection of going through the gate + yaw 2 rotations entirely 