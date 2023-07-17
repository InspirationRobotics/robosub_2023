"""
PATH CV

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

    camera = "/auv/camera/videoUSBRaw1"

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """

        logger.info("Template CV init")

    def run(self, frame, target, oakdData=None):
        """
        frame: the frame from the camera
        target: could be any type of information, for example the thing to look for
        oakd_data: only applies for oakd cameras, this is the list of detections

        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        logging.info("Template CV run")
        #frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        into_hsv =(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))
        L_limit=np.array([8, 100, 100]) 
        U_limit=np.array([50, 255, 255]) 
        # L_limit=np.array([5, 25, 50]) 
        # U_limit=np.array([30, 255, 255]) 
        #L_limit = np.array([8, 100, 100])
        #U_limit = np.array([60, 255, 255])
        # for more range
        # L_limit = np.array([3, 25, 20])
        # U_limit = np.array([50, 255, 255])
    
        orange=cv2.inRange(into_hsv,L_limit,U_limit)
    
        kernel = np.ones((5, 5), np.uint8)
        orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel)
    
        orange = cv2.GaussianBlur(orange, (11,11), 0)
        ret, thresh = cv2.threshold(orange, 230, 255, cv2.THRESH_BINARY)
    
        blur = cv2.blur(thresh, (10,10))
    
        ret2, thresh2 = cv2.threshold(blur, 1, 255, cv2.THRESH_OTSU)
        contours, heirarchy = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        yaw =0
        lateral=0
        forward=0
        hline = cv2.line(frame, (640, 0), (0, 0), (0, 255, 255), 8)
        if contours:
            c = max(contours, key = cv2.contourArea)
            cv2.drawContours(frame, c, -1, 255, 3)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(frame, "center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
            rows,cols = frame.shape[:2]
            [vx,vy,x,y] = cv2.fitLine(c, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            cv2.line(frame,(cols-1,righty),(0,lefty),(0,255,0),2)
            cv2.line(frame,(righty,cols-1),(lefty, 0),(0,255,0),2)
            slope = (lefty-righty)/(0-(cols-1))
            print(slope)
            if(slope == 0):
                slope = slope
            else:
                slope = -1/slope
            #cv2.putText(frame, str(slope), (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            if(-20 < slope < 0):
                print("yaw clockwise")
                yaw=1
            elif (0 < slope < 20):
                print("yaw counter-clockwise")
                yaw=-1
            elif(slope == 0):
                #strafe left or right to align
                print("forward now")
                forward=1
    
        return {"lateral": 0, "forward": forward,"yaw":yaw, "end": False}, frame

