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

    camera = "/auv/camera/videoOAKdRaw0"

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """
        
        # only applies for oakd cameras, if using USB, ignore this
        self.oakd_data = None
        self.oakd_data_received = False
        
        self.stepOne = False
        self.maxGlyphLength = 0
        self.stepTwo = False
        self.stepThree = False
        logger.info("Template CV init")

    def run(self, frame, target, detections):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        if target == 0, abydos
        if target == 1, earth
        """
        logging.info("Template CV run")

        a = 18*[1500]
        if(len(detections)!=0):
            # If the frame is available, draw bounding boxes on it and show the frame
            height = frame.shape[0]
            width  = frame.shape[1]
            centerOfGate = -1
            targetGate = -1
            sumOfDets = 0
            targetConfidences = []
            maxConfidence = 0
            for detection in detections:
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                sumOfDets+=detection.xmin*width
                cv2.putText(frame, str(detection.label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), cv2.FONT_HERSHEY_SIMPLEX)
                if(detection.label == target):
                   cv2.rectangle(frame, (x1, y1), (x2, y2), (0,0,255), cv2.FONT_HERSHEY_SIMPLEX)
                   targetConfidences.append((detection.confidence,x1))
            for confidence in targetConfidences:
                if confidence[0]>maxConfidence:
                    maxConfidence = confidence[0]
                    targetGate = confidence[1]
            if(len(detections)!=0):
                centerOfGate = int(sumOfDets/len(detections))
                for i in range(300):
                    frame[i][centerOfGate] = (255,255,255)
                #Feedback Loop
                #strafe until we hit the center of the gate (ensure that we don't lose teh image)
                #parallel to abydos (yaw until the length of highesgt confidence detection is longest, continue moving until it gets smaller)
                #strafe until we hit the center of the highest confidence glyph
                tolerance = 10
                if(self.stepOne== False):
                    if(centerOfGate!=-1):
                        if(centerOfGate<320-tolerance):
                            print("strafe right")
                            a[5]=1580
                        elif(centerOfGate>320+tolerance):
                            print("strafe left")
                            a[5]=1420
                        else:
                            print("aligned, continue")
                            self.stepOne = True 
                if(self.stepOne):
                    if(targetGate!=-1):
                        for i in range(300):
                            frame[i][targetGate] = (0,0,255)
                            if(targetGate<320-tolerance):
                                print("turn left")
                                a[3]=1450
                            elif(targetGate>320+tolerance):
                                print("turn right")
                                a[3]=1550
                            else:
                                print("aligned, continue")
                            lengthOfGlyph = x2-x1
                            if(lengthOfGlyph<maxGlyphLength):
                                self.stepTwo = True
                            else:
                                maxGlyphLength = lengthOfGlyph
                if(self.stepTwo):
                    if(targetGate!=-1):
                        if(targetGate<320-tolerance):
                            print("strafe right")
                            a[5]=1580
                        elif(targetGate>320+tolerance):
                            print("strafe left")
                            a[5]=1420
                        else:
                            print("aligned, continue")
                            self.stepThree = True
                if(self.stepThree):
                    print("go forward")
                    a[4]=1580
        print(a)
        return {"lateral": a[5], "forward": a[4],"yaw":a[3], "end": False}, frame
