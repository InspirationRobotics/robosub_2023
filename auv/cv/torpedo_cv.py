"""
Description: CV torpedo class
Author: Kyle Jacob
"""


import logging
import time
import cv2
import numpy as np
import argparse
import logging
import os
import time

from auv.device.sonar import Ping360, utils, io

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class CV:
    camera = "/auv/camera/videoUSBRaw0"

    def __init__(self, **config):
        """
        Init of torpedo CV,
        """

        self.frame = None
        self.lostSight = 0
        logger.info("Torpedo CV init")

        self.target_center_x = 210      # Where we want the center of the circle
        self.target_center_y = 350      # to be when close to mission

        self.center_x = 240             # True center of screen which will be
        self.center_y = 320             # used to align the sub when far away

        self.threshold_far = 30         # Margin of error when far
        self.threshold_near = 50        # "      "      " when near
        self.scan_timer = 0
        self.distance_to_target = None

        # TODO Fill with distance boundary that we will get from ping 360 to
        # switch from center the center of the sub the the largest circle, to
        # then centering the torpedo zone
        self.far_near_boundary = 250    # if radius > ; means near, else > means far
        self.fire_distance = 280        # if radius of largest circle is > than, fire


        step_angle = 1
        max_range = 20

        p = Ping360(
            "/dev/ttyUSB0",
            115200,
            scan_mode=1,
            angle_range=(150, 250),
            angle_step=step_angle,
            max_range=max_range,
            gain=2,
            transmit_freq=800,
    )


    def get_circles (self):
        """
        Returns the center and radius of the detected circle in the frame
        using a Hough circle detection on Canny edge detection
        """

        # Convert frame to grey scale and extract edges
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 1, 325)

        # Blurring the image to eliminate false positives
        edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
        edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)


        # Detecting Hough Circles on the image and saving them in circles
        circles = cv2.HoughCircles(edges,
                                    cv2.HOUGH_GRADIENT,
                                    1, 
                                    edges.shape[0] / 8, 
                                    param1=10, 
                                    param2=100, 
                                    minRadius=1, maxRadius=600)
        

        # Draw all circles on the original image
        if circles is not None:
            i = 0

            circles = np.round(circles[0, :]).astype("int")
            for circle in circles:
                
                x,y = circles[i,0], circles[i,1]
                dist = circles[i, 2]
                cv2.circle(self.frame, center=(x, y), radius=dist, 
                           color=(0, 255, 0), thickness=20)
                cv2.circle(self.frame, center=(circles[i, 0], circles[i, 1]), 
                           radius=2, color=(0, 0, 255), thickness=2)
                i += 1
        
        # Return list containing all circles in the frame
        return (circles)
                
        


    def run(self, frame):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        logging.info("Torpedo CV run")

        # video is 480 by 640, at end we want the point (210, 350)
        self.frame = frame

       
        circles = self.get_circles()




        # Image processing
        if circles is not None:             # Circles detected
            self.lostSight = 0

            if len(circles) <= 2:        # Can be in firing position
                print("None") 


            # State 1: Only one circle detected
            if len(circle = 1):
                # Want to align horizontal componet 
                x,y,radius = circles[0,0], circles[0,1], circles[0]
                


            if len(circles) <= 3:       # Can check for open vs closed iris
                print("NULL")


        else:   # No circles detected
            self.lostSight += 1

            if self.lostSight > 300:
                # Target has been lost for too long and mission needs to terminate
                return {"lateral": 0, 
                        "forward": 0, 
                        "vertical": 0, 
                        "end": True}, self.frame


            elif self.lostSight > 30:
                # No circles detected and need to back up looking any
                return {"lateral": 0, 
                        "forward": -5, 
                        "vertical": 0, 
                        "end": False}, self.frame
            else:
                # Lost image for a short duration so waiting to see
                # if the image will detect another circle
                return {"lateral": 0, 
                        "forward": 0, 
                        "vertical": 0, 
                        "end": False}, self.frame

            






        #return {"lateral": 0, "forward": 0, "vertical": 0, "end": False}, self.frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template_cv"
    logging.basicConfig(level=logging.INFO)

    # Create a CV object with arguments
    cv = CV()

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("testing_data\\Torpedo1.mp4")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        # run the cv
        result, img_viz = cv.run(frame)
        logger.info(result)

        # show the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
