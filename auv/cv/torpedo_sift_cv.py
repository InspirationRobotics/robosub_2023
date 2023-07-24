"""
Description: CV torpedo using sift class
Author: Maxime Ellerbach
"""

import time
import cv2
import numpy as np
import argparse
import os
import time

from auv.device.sonar import Ping360, utils, io


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

        self.reference_image = cv2.imread("auv/cv/samples/torpedo.png")
        assert self.reference_image is not None, "Could not load reference image"
        self.ref_shape = self.reference_image.shape

        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher()
        self.kp1, self.des1 = self.sift.detectAndCompute(self.reference_image, None)

    def get_3dpose_from_sift(self, img, threshold=0.65, viz=True):
        """
        Sift matching between the reference image and the current image
        returns a
        """

        kp2, des2 = self.sift.detectAndCompute(img, None)
        matches = self.bf.knnMatch(self.des1, des2, k=2)

        good = []
        for m, n in matches:
            if m.distance < threshold * n.distance:
                good.append(m)

        if len(good) < 4:
            return None

        src_pts = np.float32([self.kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        # compute the homography
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        if H is None:
            return None, None

        if viz:
            # draw the matches
            img3 = cv2.drawMatches(
                self.reference_image,
                self.kp1,
                img,
                kp2,
                good,
                None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
            )
            cv2.imshow("matches", img3)

            # viz the homography
            h, w, _ = self.reference_image.shape
            pts = np.float32([
                [0, 0],
                [0, h - 1],
                [w - 1, h - 1],
                [w - 1, 0]
            ]).reshape(-1, 1, 2)

            dst_viz = cv2.perspectiveTransform(pts, H).astype(np.int32)
            img2 = cv2.polylines(img, [dst_viz], True, 255, 3, cv2.LINE_AA)
            cv2.imshow("homography", img2)

        inv_H = np.linalg.inv(H)
        return H, inv_H

    def get_relative_coords(self, H, inv_H):
        """get position of the submarine relative to the reference image"""

        # get the center of the reference image
        h, w, _ = self.reference_image.shape
        center = np.float32([[w / 2, h / 2]]).reshape(-1, 1, 2)
        center_ref = cv2.perspectiveTransform(center, H).astype(np.int32)[0][0]

        # get 3d coords where is the sub pointing to
        center_relative = cv2.perspectiveTransform(center, inv_H).astype(np.int32)[0][0]
        center_relative = [center_relative[0] / self.ref_shape[1], center_relative[1] / self.ref_shape[0]]

        return center_relative

    def run(self, frame, target, detections):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """

        forward = 0
        lateral = 0
        yaw = 0
        end = False

        equilized = equilize(frame)
        self.img_viz = equilized.copy()

        H, inv_H = self.get_3dpose_from_sift(equilized, threshold=0.65, viz=True)
        if H is None:
            return {}, self.img_viz

        center_relative = self.get_relative_coords(H, inv_H)
        print(f"[INFO] center relative: {center_relative}")

        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, self.img_viz


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

        time.sleep(0.02)
        # run the cv
        result, img_viz = cv.run(frame, None, None)
        print(f"[INFO] {result}")

        # show the frame
        cv2.imshow("frame", frame)
        cv2.imshow("viz", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
