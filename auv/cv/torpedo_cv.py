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

from auv.device.sonar import Ping360, utils, io


class CV:
    camera = "/auv/camera/videoUSBRaw0"

    def __init__(self, **config):
        """
        Init of torpedo CV,

        """

        # TODO Change when testing
        self.deploy = True

        self.frame = None
        self.lostSight = 0
        print("[INFO] Torpedo CV init")

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
        self.far_near_boundary = 2  # if distance < 2 meteres, we are near
        self.fire_distance = 1  # if distacne < 1 meter, we are in range

        step_angle = 1
        max_range = 20

        self.p = (
            Ping360(
                "/dev/ttyUSB1",
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

    def get_circles(self, frame, p1, p2):
        """
        Returns the center and radius of the detected circle in the frame
        using a Hough circle detection on Canny edge detection
        """

        # Convert frame to grey scale and extract edges
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)

        # Blurring the image to eliminate false positives
        edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
        edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)

        # Detecting Hough Circles on the image and saving them in circles
        circles = cv2.HoughCircles(
            edges,
            cv2.HOUGH_GRADIENT,
            1,
            edges.shape[0] / 8,
            param1=p1,
            param2=p2,
            minRadius=1,
            maxRadius=600,
        )

        if not self.deploy:
            cv2.imshow("Edges", edges)

        # Draw all circles on the original image
        if circles is not None:
            i = 0

            circles = np.round(circles[0, :]).astype("int")

            for circle in circles:
                x, y = circles[i, 0], circles[i, 1]
                dist = circles[i, 2]
                cv2.circle(
                    self.frame,
                    center=(x, y),
                    radius=dist,
                    color=(0, 255, 0),
                    thickness=20,
                )
                cv2.circle(
                    self.frame,
                    center=(circles[i, 0], circles[i, 1]),
                    radius=2,
                    color=(0, 0, 255),
                    thickness=2,
                )
                i += 1
        cv2.circle(
            self.frame,
            center=(self.center_x, self.center_y),
            radius=2,
            color=(255, 0, 0),
            thickness=3,
        )

        # Return list containing all circles in the frame
        return circles

    def run(self, frame, target, detections):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        # print("[INFO] Torpedo CV run")

        forward = 0
        lateral = 0
        yaw = 0
        end = False

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
        if self.near:
            circles = self.get_circles(frame, 50, 80)
        if not self.near:
            circles = self.get_circles(frame, 50, 65)

        # Image processing
        if circles is not None:  # Circles detected
            print("[INFO] Circle detected")

            self.lostSight = 0

            # Get distance with ping 360
            obstacles = self.p.get_obstacles() if self.deploy else None

            if obstacles is not None:
                # Sort obstacles by size
                #sorted_obstacles = sorted(obstacles, key=lambda x: x.area, reverse=True)

                # Grab the distance of the first object in the list
                try:
                    largest_object = sorted_obstacles[0]
                    self.distance_to_target = largest_object.distance
                    # print("[INFO] Distance: " + self.distance_to_target)
                    print(f"Distance: {self.distance_to_target}")

                    if self.distance_to_target < self.far_near_boundary:
                        self.near = True
                        print("[INFO] Near")
                except:
                    print(["[ERROR] Ping360 Error"])

            # Center of largest circle - aim for this
            x, y = circles[0, 0], circles[0, 1]
            print(f"[INFO] X: {str(np.round(x).astype('int'))}")
            print(f"[INFO] Y: {str(np.round(y).astype('int'))}")
            last_y = y

            # Sleep delay for letting motors move
            #time.sleep(1)

            if self.distance_to_target < self.fire_distance:
                # Fire both and dont worry about moving to different one
                print("[INFO] Fire torpedos!!!")
                self.fired_torpedo_1 = True
                return {
                    "lateral": 0,
                    "forward": 0,
                    "vertical": self.depth,
                    "fire1": True,
                    "fire2": True,
                    "end": False,
                }, frame

                # if not self.fired_torpedo_1:
                #     if self.open_is_top:
                #         self.depth += 0.1
                #     if not self.open_is_top:
                #         self.depth += 0.1
                #     print("[INFO] Fire torpedo 1!!!")
                #     self.fired_torpedo_1 = True
                #     return {
                #         "lateral": 0,
                #         "forward": 0,
                #         "vertical": self.depth,
                #         "fire1": True,
                #         "fire2": False,
                #         "end": False,
                #     }, frame
                # else:
                #     print("[INFO] Fire torpedo 2!!!")
                #     self.fired_torpedo_2 = True
                #     return {
                #         "lateral": 0,
                #         "forward": 0,
                #         "vertical": self.depth,
                #         "fire1": True,
                #         "fire2": True,
                #         "end": False,
                #     }, frame

            if not self.near:
                # Aligning from afar

                # X alignment
                if self.center_x > x + self.threshold_far:  # Strafe Left
                    print("[INFO] Left")
                    lateral = -1

                if self.center_x < x - self.threshold_far:  # Strafe Right
                    print("[INFO] Right")
                    lateral = 1

                if lateral == 0:
                    print("[INFO] Centered on X axis")


                # Y alignment
                if self.center_y < y - self.threshold_far:  # Dive
                    self.depth += 0.02
                    print("[INFO] Dive")
                if self.center_y > y + self.threshold_far:  # Ascend
                    self.depth -= 0.02
                    print("[INFO] Ascend")

                return {
                    "lateral": lateral,
                    "forward": 1,
                    "vertical": self.depth,
                    "end": False,
                }, frame

            else:
                # Aligning near the target, aim for lowest y value target
                sorted_obstacles = sorted(circles, key=lambda x: x.y, reverse=True)
                x, y = circles[0, 0], circles[0, 1]

                self.open_is_top = True if y > last_y else None

                # X alignment
                if self.target_center_x < x + self.threshold_near:  # Strafe Left
                    print("[INFO] Left")
                    lateral = -1

                if self.target_center_x > x - self.threshold_near:  # Strafe Right
                    print("[INFO] Right")
                    lateral = 1

                # Y alignment
                if self.target_center_y < y - self.threshold_near:  # Dive
                    self.depth += 0.02
                    print("[INFO] Dive")
                if self.target_center_y > y + self.threshold_near:  # Ascend
                    self.depth -= 0.02
                    print("[INFO] Ascend")

                if lateral == 0:
                    print("[INFO] X Centered")


                # Print motors and return commands
                return {
                    "lateral": lateral,
                    "forward": 1,
                    "vertical": self.depth,
                    "end": False,
                }, frame

        else:  # No circles detected
            # Potentially can resort to scanning sonar with 180 degree sweep
            # and find if the object is on the left or right of the sub
            self.lostSight += 1                    
            print("[INFO] Lost Sight for " + str(self.lostSight) + " frames")


            if self.lostSight > 3000:
                # Target has been lost for too long and mission needs to terminate
                end = True

            if self.lostSight > 400:
                # No circles detected and need to back up looking any
                forward = -1
                print("[INFO] Backing up")


            return {
                "lateral": 0,
                "forward": forward,
                "vertical": 0,
                "end": end,
            }, self.frame

        # return {"lateral": 0, "forward": 0, "vertical": 0, "end": False}, self.frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.torpedo_cv"

    # Create a CV object with arguments
    cv = CV()

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("testing_data/Torpedo1.mp4")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        time.sleep(0.1)
        # run the cv
        result, img_viz = cv.run(frame, None, None)
        print(f"[INFO] {result}")

        # show the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
