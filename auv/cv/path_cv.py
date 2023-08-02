"""
PATH CV

"""

# import what you need from within the package

import time

import cv2
import numpy as np

from ..utils import deviceHelper

class CV:
    """Template CV class, don't change the name of the class"""

    camera = "/auv/camera/videoUSBRaw1"

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """
        self.config = config
        self.config = deviceHelper.variables
        self.aligned = False
        self.state = "strafe"
        self.current_sub = self.config.get("sub", "graey")
        if self.current_sub == "graey":
            self.camera = "/auv/camera/videoUSBRaw1"
        elif self.current_sub == "onyx":
            self.camera = "/auv/camera/videoOAKdRawBottom"
        print("[INFO] Template CV init")

    def equalizeHist(self, frame):
        frame_b, frame_g, frame_r = cv2.split(frame)
        frame_g = cv2.equalizeHist(frame_g)
        frame_b = cv2.equalizeHist(frame_b)
        frame_r = cv2.equalizeHist(frame_r)
        frame = cv2.merge((frame_b, frame_g, frame_r))
        return frame

    def alignMidpoint(self, midpoint, tolerance):
        if midpoint < 320 - tolerance:
            lateral = -1
        elif midpoint > 320 + tolerance:
            lateral = 1
        else:
            lateral=0
        
        return lateral
    def run(self, frame, target, oakdData=None):
        """
        frame: the frame from the camera
        target: could be any type of information, for example the thing to look for
        oakd_data: only applies for oakd cameras, this is the list of detections

        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        end = False
        # into_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # frame = self.equalizeHist(frame)
        
        # filter the image to orange objects, filters what is white
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        into_hsv =(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))
        L_limit=np.array([3, 20, 20]) 
        U_limit=np.array([80, 255, 255])
        # L_limit=np.array([5, 25, 50])
        # U_limit=np.array([30, 255, 255])
        # racquet club testing:
        # L_limit = np.array([8, 100, 100])
        # U_limit = np.array([50, 255, 255])
        # for more range
        # L_limit = np.array([3, 25, 20])
        # U_limit = np.array([50, 255, 255])
        kernel = np.ones((5, 5), np.uint8)

        orange=cv2.inRange(into_hsv,L_limit,U_limit)
        edges = cv2.Canny(gray, threshold1=100, threshold2=200)
        ret, thresh3 = cv2.threshold(edges, 230, 255, cv2.THRESH_BINARY)
        ret2, thresh3 = cv2.threshold(thresh3, 1, 255, cv2.THRESH_OTSU)
        # cv2.imshow("edges", thresh3)
        # Removing Noise
        orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel)
        time.sleep(0.2)
        # Blur and Threshold 
        orange = cv2.GaussianBlur(orange, (11, 11), 0)
        ret, thresh = cv2.threshold(orange, 230, 255, cv2.THRESH_BINARY)

        blur = cv2.blur(thresh, (10, 10))
        ret2, thresh2 = cv2.threshold(blur, 1, 255, cv2.THRESH_OTSU)

        # Find Contours
        contours, heirarchy = cv2.findContours(thresh3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, 255, 3)

        yaw = lateral = forward = 0
        hline = cv2.line(frame, (640, 0), (0, 0), (0, 255, 255), 8)

        # Path Found
        if contours:
            c = max(contours, key=cv2.contourArea)
            cv2.drawContours(frame, c, -1, 255, 3)

            # Find Center
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(frame, "center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            rows, cols = frame.shape[:2]

            # Fit Line to Path
            [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
            lefty = int((-x * vy / vx) + y)
            righty = int(((cols - x) * vy / vx) + y)
            cv2.line(frame, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)
            cv2.line(frame, (righty, cols - 1), (lefty, 0), (0, 255, 0), 2)

            # Calculate slope
            slope = (lefty - righty) / (0 - (cols - 1))
            # cv2.putText(frame, str(slope), (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            dist = abs(cx - 320) / 320
            if(self.state == "strafe"):
                lateral = self.alignMidpoint(cx, 20)
                if(lateral == 0):
                    self.state = "yaw"
                else:
                    lateral = np.clip(lateral * dist * 4, -1, 1)
            # Movement Decisions
            elif(self.state == "yaw"):
                if -10 < slope < 0:
                    print("yaw clockwise")
                    yaw = 0.65
                elif 0 < slope < 10:
                    print("yaw counter-clockwise")
                    yaw = -0.65
                else:
                    forward=2
                
        elif self.aligned == True:
            end = True
        else:
            forward = 1

        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.surfacing_cv"

    # Create a CV object with arguments
    cv = CV()

    cap = cv2.VideoCapture("../../testing_data/path_footage.MKV")

    while True:
        ret, img = cap.read()
        if not ret:
            break

        # set the frame
        img = cv2.resize(img, (640, 480))

        # run the CV
        result, img_viz = cv.run(img, None, None)

        # show the result
        cv2.imshow("frame", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
