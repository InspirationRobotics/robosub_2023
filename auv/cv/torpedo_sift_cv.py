"""
Description: CV torpedo using sift class
Author: Maxime Ellerbach
"""

import argparse
import math
import os
import time

import cv2
import numpy as np

from ..device.sonar import Ping360, io, utils


def equilize(img):
    """Equilize the histogram of the image"""
    b, g, r = cv2.split(img)
    b = cv2.equalizeHist(b)
    g = cv2.equalizeHist(g)
    r = cv2.equalizeHist(r)
    return cv2.merge((b, g, r))


class CV:
    camera = "/auv/camera/videoUSBRaw0"

    def __init__(self, **config):
        """
        Init of torpedo CV,
        """

        self.reference_image_c = cv2.imread("auv/cv/samples/torpedo_closed.png")
        self.reference_image_o = cv2.imread("auv/cv/samples/torpedo_opened.png")
        self.reference_image_top_opened = cv2.imread("auv/cv/samples/torpedo_top_opened.png")
        self.reference_image_top_closed = cv2.imread("auv/cv/samples/torpedo_top_closed.png")
        assert self.reference_image_c is not None
        assert self.reference_image_o is not None
        assert self.reference_image_top_opened is not None
        assert self.reference_image_top_closed is not None
        self.ref_c_shape = self.reference_image_c.shape
        self.ref_o_shape = self.reference_image_o.shape

        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher()
        self.kp1_c, self.des1_c = self.sift.detectAndCompute(self.reference_image_c, None)
        self.kp1_o, self.des1_o = self.sift.detectAndCompute(self.reference_image_o, None)

        self.kp1, self.des1 = None, None  # will be calculated later on
        self.reference_image = None  # will be determined later on

        self.shape = (480, 640, 3)
        self.step = 0

        # keep track of the position of the open / closed torpedo
        self.center_c = []
        self.center_o = []
        self.on_top = None

        # absolute, normalized
        self.offset_center = 0.25
        self.target_coords = (0.0, 0.0)

    def process_sift(self, ref_img, img, kp1, des1, kp2=None, des2=None, threshold=0.65, window_viz=None):
        """
        Sift matching with reference points
        if kp2 and des2 are not given, it will compute them
        """
        if kp2 is None or des2 is None:
            kp2, des2 = self.sift.detectAndCompute(img, None)

        matches = self.bf.knnMatch(des1, des2, k=2)
        good = []
        for m, n in matches:
            if m.distance < threshold * n.distance:
                good.append(m)

        if len(good) < 4:
            return None

        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        if window_viz is not None:
            # draw the matches
            img = cv2.drawMatches(
                ref_img,
                kp1,
                img,
                kp2,
                good,
                None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
            )
            cv2.imshow(window_viz, img)
        return H

    def get_center(self, H, src_shape, norm=False):
        """Get center of a given homography H"""
        h, w, _ = src_shape
        center = np.array([[w / 2, h / 2]]).reshape(-1, 1, 2)
        center = cv2.perspectiveTransform(center, H).astype(np.int32)[0][0]
        if norm:
            center = [(center[0] - 320) / 320, (center[1] - 240) / 240]
        return center

    def get_orientation(self, H, src_shape):
        h, w, _ = src_shape
        pts = (
            np.array(
                [
                    [0, 0],
                    [w, 0],
                    [w, h],
                    [0, h],
                ]
            )
            .reshape(-1, 1, 2)
            .astype(np.float32)
        )
        pts = cv2.perspectiveTransform(pts, H).astype(np.int32).reshape(4, 2)

        # get an estimation of the Y axis rotation
        left_h_dist = np.linalg.norm(pts[0] - pts[3])
        right_h_dist = np.linalg.norm(pts[1] - pts[2])

        # TODO find the right formula for this
        yaw = math.atan((left_h_dist - right_h_dist) / (left_h_dist + right_h_dist))
        yaw = math.degrees(yaw)
        print(f"[INFO] Yaw: {yaw}")
        return yaw

    def init_find_both_centers(self, img, threshold=0.65, window_viz=None):
        """Get sift for both opened and closed torpedo"""
        kp2, des2 = self.sift.detectAndCompute(img, None)
        H_c = self.process_sift(self.reference_image_c, img, self.kp1_c, self.des1_c, kp2, des2, threshold, "H_c")
        H_o = self.process_sift(self.reference_image_o, img, self.kp1_o, self.des1_o, kp2, des2, threshold, "H_o")

        if H_c is None or H_o is None:
            return None, None

        center_c = self.get_center(H_c, self.ref_c_shape)
        center_o = self.get_center(H_o, self.ref_o_shape)

        if window_viz is not None:
            # draw the center
            cv2.circle(img, (int(center_c[0]), int(center_c[1])), 5, (0, 255, 0), -1)
            cv2.circle(img, (int(center_o[0]), int(center_o[1])), 5, (0, 255, 0), -1)
            cv2.imshow(window_viz, img)

        return center_c, center_o

    def run(self, frame, target, detections):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """

        frame = equilize(frame)

        forward = 0
        lateral = 0
        yaw = 0
        end = False

        # find setup (closed or opened on top)
        if self.step == 0:
            center_c, center_o = self.init_find_both_centers(frame, window_viz="centers")
            if center_c is None or center_o is None:
                # skip (maybe go forward a bit)
                return {}, None

            # calculate mean of the centers
            if len(self.center_c) > 10 and len(self.center_o) > 10:
                mean_c = np.mean(self.center_c, axis=0)
                mean_o = np.mean(self.center_o, axis=0)

                # determine which one is on top
                if mean_c[1] < mean_o[1]:
                    # closed is on top
                    self.on_top = "closed"
                    self.reference_image = self.reference_image_top_closed
                else:
                    # opened is on top
                    self.on_top = "opened"
                    self.reference_image = self.reference_image_top_opened

                # compute kp1 des1 for the desired ref image
                self.kp1, self.des1 = self.sift.detectAndCompute(self.reference_image, None)
                print(f"[INFO] {self.on_top} is on top")

                # cleanup and go to next step
                self.center_c = []
                self.center_o = []
                self.reference_image_c = None
                self.reference_image_o = None
                self.kp1_c, self.des1_c = None, None
                self.kp1_o, self.des1_o = None, None
                cv2.destroyAllWindows()

                self.step = 1

            else:
                self.center_c.append(center_c)
                self.center_o.append(center_o)

        # yaw and lateral to align with the torpedo
        elif self.step == 1:
            H = self.process_sift(self.reference_image, frame, self.kp1, self.des1, threshold=0.65, window_viz="H")
            if H is None:
                # skip (maybe go forward a bit)
                return {}, None

            center = self.get_center(H, self.reference_image.shape)
            yaw = self.get_orientation(H, self.reference_image.shape)
            # print(f"[INFO] Yaw: {yaw}")

            frame = cv2.circle(frame, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)

            if self.on_top == "closed":
                self.target_coords = (0.0, -self.offset_center)

        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.torpedo_cv"

    # Create a CV object with arguments
    cv = CV()

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("testing_data/Torpedo2.mp4")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        time.sleep(0.02)
        # run the cv
        result, img_viz = cv.run(frame, None, None)
        # print(f"[INFO] {result}")

        # show the frame
        if img_viz is not None:
            cv2.imshow("viz", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
