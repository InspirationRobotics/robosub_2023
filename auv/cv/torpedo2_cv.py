"""
Description: CV torpedo class
Author: Kyle Jacob
"""

import time
import cv2
import numpy as np
import argparse

import os
import time
import matplotlib.pyplot as plt


from auv.device.sonar import Ping360, utils, io


class CV:
    camera = "/auv/camera/videoUSBRaw0"

    def __init__(self, **config):
        """
        Init of torpedo CV,

        """

        # TODO Change when testing
        self.deploy = False

        self.frame = None
        self.lostSight = 0
        print("[INFO] Torpedo CV init")

        # self.camera = "/auv/camera/videoUSBRaw0"

        self.depth = 0.35

        self.target_center_x = 240  # Where we want the center of the circle
        self.target_center_y = 320  # to be when close to mission

        self.center_x = 330  # True center of screen which will be
        self.center_y = 250  # used to align the sub when far away

        self.threshold_far = 30  # Margin of error when far
        self.threshold_near = 50  # "      "      " when near
        self.distance_to_target = 9999
        self.near = False

        self.fired_torpedo_1 = False
        self.fired_torpedo_2 = False

        self.open_is_top = None

        # TODO Fill with distance boundary that we will get from ping 360 to
        # switch from center the center of the sub the the largest circle, to
        # then centering the torpedo zone
        self.far_near_boundary = 2.5  # if distance < 2 meteres, we are near
        self.fire_distance = 1  # if distacne < 1 meter, we are in range

        step_angle = 1
        max_range = 20

        self.p = (
            Ping360(
                "/dev/ttyUSB0",
                115200,
                scan_mode=1,
                angle_range=(150, 250),
                angle_step=step_angle,
                max_range=max_range,
                gain=2,
                transmit_freq=800,
            )
            if self.deploy
            else None
        )

    def get_sift(self, frame):
        """
        Returns the center and radius of the detected circle in the frame
        using a Hough circle detection on Canny edge detection
        """

        img1 = cv2.imread("testing_data/Torp_IMG.png")#,cv2.IMREAD_GRAYSCALE) # queryImage
        img2 = cv2.imread("testing_data/Torp_ENV.png")#,cv2.IMREAD_GRAYSCALE) # trainImage

        # equalize channels
        frame_b, frame_g, frame_r = cv2.split(img1)
        frame_g = cv2.equalizeHist(frame_g)
        frame_b = cv2.equalizeHist(frame_b)
        frame_r = cv2.equalizeHist(frame_r)
        img1 = cv2.merge((frame_b, frame_g, frame_r))


        # equalize channels
        frame_b, frame_g, frame_r = cv2.split(img2)
        frame_g = cv2.equalizeHist(frame_g)
        frame_b = cv2.equalizeHist(frame_b)
        frame_r = cv2.equalizeHist(frame_r)
        img2 = cv2.merge((frame_b, frame_g, frame_r))


        # Initiate SIFT detector
        sift = cv2.SIFT_create()
        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(img1,None)
        kp2, des2 = sift.detectAndCompute(img2,None)
        # BFMatcher with default params
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1,des2,k=2)
        # Apply ratio test
        good = []
        for m,n in matches:
            if m.distance < 0.55*n.distance:
                good.append([m])
        # cv.drawMatchesKnn expects list of lists as matches.
        img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        plt.imshow(img3),plt.show()


    def get_orb(self,frame):
        img1 = cv2.imread("testing_data/Torp_IMG.png")#,cv2.IMREAD_GRAYSCALE) # queryImage
        #img2 = cv2.imread("testing_data/Torp_IMG.png",cv2.IMREAD_GRAYSCALE) # queryImage

        img2 = cv2.imread("testing_data/T.png")#,cv2.IMREAD_GRAYSCALE) # trainImage
        # Initiate ORB detector
        orb = cv2.ORB_create()
        # find the keypoints and descriptors with ORB
        kp1, des1 = orb.detectAndCompute(img1,None)
        kp2, des2 = orb.detectAndCompute(img2,None)
        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        # Match descriptors.
        matches = bf.match(des1,des2)
        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)
        # Draw first 10 matches.
        img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:20],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        plt.imshow(img3),plt.show()


        

    def run(self, frame):
        """
        Here should be all the code required to run the cv2.
        This could be a loop, grabing frames using ROS, etc.
        """
        # print("[INFO] Torpedo CV run")

        if self.fired_torpedo_1 and self.fired_torpedo_2:
            print("[INFO] Mission complete!!")
            return {
                "lateral": 0,
                "forward": 0,
                "vertical": self.depth,
                "fire1": True,
                "fire2": True,
                "end": True,
            }, frame

        # video is 480 by 640, at end we want the point approx. (210, 350)
        self.frame = frame
        self.get_sift(frame)

        #self.get_orb(frame)
        # print("here")
        return {"lateral": 0, "forward": 0, "vertical": 0, "end": False}, self.frame

        # Image processing
        # if circles is not None:  # Circles detected
        #     print("[INFO] Circle detected")

        #     self.lostSight = 0

        #     # Get distance with ping 360
        #     obstacles = self.p.get_obstacles() if self.deploy else None

        #     if obstacles is not None:
        #         # Sort obstacles by size
        #         sorted_obstacles = sorted(obstacles, key=lambda x: x.area, reverse=True)

        #         # Grab the distance of the first object in the list
        #         largest_object = sorted_obstacles[0]
        #         self.distance_to_target = largest_object.distance
        #         print("[INFO] Distance: " + self.distance_to_target)

        #         if self.distance_to_target < self.far_near_boundary:
        #             self.near = True
        #             print("[INFO] Near")

        #     # Center of largest circle - aim for this
        #     x, y = circles[0, 0], circles[0, 1]
        #     print("[INFO] X: " + str(np.round(x).astype("int")))
        #     print("[INFO] Y: " + str(np.round(y).astype("int")))
        #     last_y = y

        #     if self.distance_to_target < self.fire_distance:
        #         if not self.fired_torpedo_1:

        #             if self.open_is_top:
        #                 self.depth += 0.1
        #             if not self.open_is_top:
        #                 self.depth += 0.1
        #             print("[INFO] Fire torpedo 1!!!")
        #             self.fired_torpedo_1 = True
        #             return {
        #                 "lateral": 0,
        #                 "forward": 0,
        #                 "vertical": self.depth,
        #                 "fire1": True,
        #                 "fire2": False,
        #                 "end": False,
        #             }, frame
        #         else:
        #             print("[INFO] Fire torpedo 2!!!")
        #             self.fired_torpedo_2 = True
        #             return {
        #                 "lateral": 0,
        #                 "forward": 0,
        #                 "vertical": self.depth,
        #                 "fire1": True,
        #                 "fire2": True,
        #                 "end": False,
        #             }, frame

        #     if not self.near:
        #         # Aligning from afar

        #         # X alignment
        #         if self.center_x > x - self.threshold_far:  # Strafe Left
        #             print("[INFO] Left")
        #             return {
        #                 "lateral": -1,
        #                 "forward": 0,
        #                 "vertical": self.depth,
        #                 "end": False,
        #             }, frame
        #         if self.center_x < x + self.threshold_far:  # Strafe Right
        #             print("[INFO] Right")
        #             return {
        #                 "lateral": 1,
        #                 "forward": 0,
        #                 "vertical": self.depth,
        #                 "end": False,
        #             }, frame

        #         # Y alignment
        #         if self.center_y < y - self.threshold_far:  # Dive
        #             self.depth += 0.02
        #             print("[INFO] Dive")
        #         if self.center_y > y + self.threshold_far:  # Ascend
        #             self.depth -= 0.02
        #             print("[INFO] Ascend")

        #         return {
        #             "lateral": 1,
        #             "forward": 1,
        #             "vertical": self.depth,
        #             "end": False,
        #         }, frame

        #     else:
        #         # Aligning near the target, aim for lowest y value target
        #         sorted_obstacles = sorted(circles, key=lambda x: x.y, reverse=True)
        #         x, y = circles[0, 0], circles[0, 1]

        #         self.open_is_top = True if y > last_y else None

        #         # X alignment
        #         if self.target_center_x < x - self.threshold_near:  # Strafe Left
        #             print("[INFO] Left")
        #             return {
        #                 "lateral": -1,
        #                 "forward": 0,
        #                 "vertical": self.depth,
        #                 "end": False,
        #             }, frame

        #         if self.target_center_x > x + self.threshold_near:  # Strafe Right
        #             print("[INFO] Right")
        #             return {
        #                 "lateral": 1,
        #                 "forward": 0,
        #                 "vertical": self.depth,
        #                 "end": False,
        #             }, frame

        #         # Y alignment
        #         if self.target_center_y < y - self.threshold_near:  # Dive
        #             self.depth += 0.02
        #             print("[INFO] Dive")
        #         if self.target_center_y > y + self.threshold_near:  # Ascend
        #             self.depth -= 0.02
        #             print("[INFO] Ascend")

        #         # Print motors and return commands
        #         return {
        #             "lateral": 0,
        #             "forward": 0,
        #             "vertical": self.depth,
        #             "end": False,
        #         }, frame

        # else:  # No circles detected
        #     # Potentially can resort to scanning sonar with 180 degree sweep
        #     # and find if the object is on the left or right of the sub
        #     self.lostSight += 1

        #     if self.lostSight > 300:
        #         # Target has been lost for too long and mission needs to terminate
        #         return {
        #             "lateral": 0,
        #             "forward": 0,
        #             "vertical": 0,
        #             "end": True,
        #         }, self.frame

        #     elif self.lostSight > 3000:
        #         # No circles detected and need to back up looking any
        #         return {
        #             "lateral": 0,
        #             "forward": -1,
        #             "vertical": 0,
        #             "end": False,
        #         }, self.frame
        #     else:
        #         # Lost image for a short duration so waiting to see
        #         # if the image will detect another circle
        #         return {
        #             "lateral": 0,
        #             "forward": 0,
        #             "vertical": 0,
        #             "end": False,
        #         }, self.frame

        # return {"lateral": 0, "forward": 0, "vertical": 0, "end": False}, self.frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template_cv"

    # Create a CV object with arguments
    cv = CV()

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("testing_data/Torpedo4.mp4")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        #time.sleep(0.01)
        # run the cv
        result, img_viz = cv.run(frame)
        # logger.info(result)

        # show the frame
        cv2.imshow("frame", frame)

        break 
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
