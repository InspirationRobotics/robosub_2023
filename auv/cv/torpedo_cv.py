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


file_dir = os.path.dirname(os.path.abspath(__file__))


class CV:
    camera = "/auv/camera/videoUSBRaw0"

    def __init__(self, **config):
        """
        Init of torpedo CV,
        """

        self.reference_image_c = cv2.imread(f"{file_dir}/samples/torpedo_closed.png")
        self.reference_image_o = cv2.imread(f"{file_dir}/samples/torpedo_opened.png")
        self.reference_image_top_opened = cv2.imread(f"{file_dir}/samples/torpedo_top_opened.png")
        self.reference_image_top_closed = cv2.imread(f"{file_dir}/samples/torpedo_top_closed.png")
        assert self.reference_image_c is not None
        assert self.reference_image_o is not None
        assert self.reference_image_top_opened is not None
        assert self.reference_image_top_closed is not None
        self.ref_c_shape = self.reference_image_c.shape
        self.ref_o_shape = self.reference_image_o.shape

        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
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
        self.offset_center = 0.5
        self.target_coords = (0.0, 0.0)
        self.yaw_threshold = 4
        self.x_threshold = 0.1
        self.y_threshold = 0.1

        self.counter = 0
        self.aligned = True
        self.firing_range = 950
        self.near_range = 750
        self.fired1 = False
        self.fired2 = False

        print("[INFO] Torpedo cv Init")

    def equalize_clahe(self, image):
        """Equalize the histogram of the image"""
        b, g, r = cv2.split(image)
        b = self.clahe.apply(b)
        g = self.clahe.apply(g)
        r = self.clahe.apply(r)
        return cv2.merge((b, g, r))

    def equilize(img):
        """Equilize the histogram of the image"""
        b, g, r = cv2.split(img)
        b = cv2.equalizeHist(b)
        g = cv2.equalizeHist(g)
        r = cv2.equalizeHist(r)
        return cv2.merge((b, g, r))

    def process_sift(
        self,
        ref_img,
        img,
        kp1,
        des1,
        kp2=None,
        des2=None,
        threshold=0.65,
        window_viz=None,
    ):
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

    def projection(self, H, src_shape, points_normalized):
        """
        Get projection of a given homography H
        Points are normalized between -1 and 1
        """
        h, w, _ = src_shape
        points = np.array(points_normalized)
        points[:, 0] = points[:, 0] * w // 2 + w // 2
        points[:, 1] = points[:, 1] * h // 2 + h // 2
        points = points.reshape(-1, 1, 2).astype(np.float32)
        points = cv2.perspectiveTransform(points, H).astype(np.int32).reshape(-1, 2)
        return points
        
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
        width = w / np.linalg.norm(pts[0] - pts[1])

        yaw = math.atan((left_h_dist - right_h_dist) / (left_h_dist + right_h_dist))
        yaw = math.degrees(yaw)
        print(f"[INFO] Yaw: {yaw}")
        return yaw, width

    def init_find_both_centers(self, img, threshold=0.65, window_viz=None):
        """Get sift for both opened and closed torpedo"""
        kp2, des2 = self.sift.detectAndCompute(img, None)
        H_c = self.process_sift(
            self.reference_image_c,
            img,
            self.kp1_c,
            self.des1_c,
            kp2,
            des2,
            threshold,
            "M",
        )
        H_o = self.process_sift(
            self.reference_image_o,
            img,
            self.kp1_o,
            self.des1_o,
            kp2,
            des2,
            threshold,
            "N",
        )

        if H_c is None or H_o is None:
            return None, None

        center_c = self.get_center(H_c, self.ref_c_shape)
        center_o = self.get_center(H_o, self.ref_o_shape)

        if window_viz is not None:
            # draw the center
            cv2.circle(img, (int(center_c[0]), int(center_c[1])), 5, (0, 255, 0), -1)
            cv2.circle(img, (int(center_o[0]), int(center_o[1])), 5, (0, 255, 0), -1)
            # cv2.imshow(window_viz, img)

        return center_c, center_o
    

    def align_yaw(self, H, ref_shape):

        yaw, dist = self.get_orientation(H, ref_shape)
        yaw_required = (self.yaw_threshold < yaw < -self.yaw_threshold)
        if yaw_required:
            yaw = np.clip(yaw * 1, -1, 1)
        else:
            # 2 step verification to eliminate one off outlier values
            yaw = 0
            if self.aligned:
                self.step = 2
                self.aligned = False
            else:
                self.aligned = True
        return yaw_required, yaw, dist
    
    def align_lateral(self, target):
        lateral = np.clip(target[0] * 3.5, -1, 1)
        lateral_required = (self.x_threshold < lateral < -self.x_threshold)
        if not lateral_required:
            lateral = 0
            # 2 step verification to eliminate one off outlier values
            if self.aligned:
                self.step = 3
                self.aligned = False
            else:
                self.aligned = True
        return lateral_required, lateral
    
    def align_depth(self, target):
        vertical = np.clip(target[1] * 2, -0.2, 0.2)
        vertical_required = (self.y_threshold < vertical < -self.y_threshold)
        if not vertical_required:
            vertical = 0
            # 2 step verification to eliminate one off outlier values
            if self.aligned:
                self.step = 4
                self.aligned = False
            else:
                self.aligned = True
        return vertical_required, vertical
    
    
    def move_forward(self, H, ref_shape, target):
        yaw_required, yaw, dist = self.align_yaw(H, self.reference_image.shape)
        lateral_required, lateral = self.align_lateral(target)
        vertical_required, vertical = self.align_depth(target)

        # Control loop for moving forward
        if not(yaw_required or lateral_required or vertical_required):
            if dist < self.firing_range:
                forward = 1
            elif dist > self.firing_range + 200:
                # Too far, move back
                forward = -1
            return 0, 0, 0, forward


        else:
            if yaw_required:
                return yaw, 0, 0, 0
            elif lateral_required:
                return 0, lateral, 0, 0
            elif vertical_required:
                return 0, 0, vertical, 0
            if yaw_required:
                return yaw, 0, 0, 0
                


    def run(self, frame, target, detections):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        print("~~~~~~~~~~~~~~~~~~")

        frame = self.equalize_clahe(frame)

        forward = 0
        lateral = 0
        vertical = 0
        yaw = 0
        end = False
        
        message = "Looking"
        target = "Open"

        # Find setup (closed or opened on top)
        if self.step == 0:
            center_c, center_o = self.init_find_both_centers(frame, window_viz="centers")
            if center_c is None or center_o is None:
                # skip (maybe go forward a bit)
                return {}, frame
            
            # calculate mean of the centers
            print("Here")
            if len(self.center_c) > 10 and len(self.center_o) > 10:
                mean_c = np.mean(self.center_c, axis=0)
                mean_o = np.mean(self.center_o, axis=0)
                print("Marco")

                # determine which one is on top
                if mean_c[1] < mean_o[1]:
                    # closed is on top
                    self.on_top = "closed"
                    self.reference_image = self.reference_image_top_closed
                    self.offset_center = 0.8
                else:
                    # opened is on top
                    self.on_top = "open"
                    self.reference_image = self.reference_image_top_opened
                    self.offset_center = -0.8


                # compute kp1 des1 for the desired ref image
                self.kp1, self.des1 = self.sift.detectAndCompute(self.reference_image, None)
                print(f"[INFO] {self.on_top} is on top")
                cv2.destroyAllWindows()
                self.step = 1

            else:
                print("else")
                self.center_c.append(center_c)
                self.center_o.append(center_o)
                print("final")

            # Modifications
            
            """
                Step One: Yaw to align angle
                Step Two: Strafe to align lateral 
                Step Three: Dive to align vertical 
                Step Four: Drive forward and maintain Yaw, Strafe, and Depth
                    Most likely case is that when we drive forward, the target will adjust lower and need to dive more.  Yaw and lateral shouldn't change much
            
            """
        else:
            print("Here2")
            center = self.get_center(H, self.reference_image.shape, norm=True)
            if (self.on_top == open):
                center[1] += self.offset_center
            else:
                center[1] -= self.offset_center
           

            H = self.process_sift(
                self.reference_image,
                frame,
                self.kp1,
                self.des1,
                threshold=0.65,
                window_viz="H",
            )

            if center is None:
                return {}, frame
            target = [center[0], int((center[1] - self.offset_center) * 240 + 240)]
            #target = [center[0], center[1]]

            # Step 1: Align yaw
            if self.step == 1:
                message = "ALIGNING YAW"
                if H or self.reference_image is None:
                    return {}, frame
                yaw_required, yaw, dist = self.align_yaw(H, self.reference_image.shape)

            # Step 2: Align lateral strafe
            elif self.step == 2:
                message = "ALIGNING LATERAL"
                lateral_required, lateral = self.align_lateral(center)
            
            # Align vertical componet
            elif self.step == 3:
                message = "ALIGNING VERTICAL"
                vertical_required, vertical = self.align_depth(center)
                
            # Move forward and maintain depth with target, and maybe yaw or lateral.
            elif self.step == 4:
                message = "MOVING FORWARD"
                yaw, lateral, vertical, forward = self.move_forward(H, self.reference_image.shape, center)

            # Fire torpedo 1, and adjust vertical to align for next shot
            elif self.step == 5:
                message = "FIRE TORPEDO 1"
                if(self.fired1):
                    time.sleep(2)
                    if(self.on_top == "closed"):
                        self.offset_center = -0.8
                    else:
                        self.offset_center = 0.8
                    self.step = 6
                else:
                    self.fired1 = True
                    target = "Closed"

            # Sleep then fire torpedo 2
            elif self.step == 6:
                vertical_required, vertical = self.align_depth(center)
                message = "ALIGNING VERTICAL FOR SECOND TORPEDO"
                if not vertical_required:
                    if self.aligned == True:
                        # Fire torpedo 2
                        message = "FIRE TORPEDO 2"
                        self.fired2 = True
                        self.step = 7
                        
                    else:
                        self.aligned = True
                
            # End mission  
            elif self.step == 7:
                message = "END MISSION"
                time.sleep(2)
                end = True
            


            # Adding message to screen:

            # center = self.get_center(H, self.reference_image.shape, norm=True)
            # projected_targets = self.projection(
            #     H,
            #     self.reference_image.shape,
            #     [
            #         [0, self.offset_center],
            #         [0, -self.offset_center],
            #     ],
            # )

            # if self.on_top == "open":
            #     if self.fired1:
            #         target = projected_targets[0]
            #     else:
            #         target = projected_targets[1]
            # if self.on_top == "closed":
            #     if self.fired1:
            #         target = projected_targets[1]
            #     else:
            #         target = projected_targets[0]

            # # REVIEW: doesn't work like that, you have to project the offset if you want to get the position of the offset
            
            cv2.circle(
                frame,
                center=(target[0], target[1]),
                radius=1,
                color=(0, 0, 255),
                thickness=3,
            )
            cv2.circle(
                frame,
                center=(320, 240),
                radius=1,
                color=(0, 255, 0),
                thickness=3,
            )

            cv2.putText(frame, message, (220, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            return {
                "lateral": lateral,
                "forward": forward,
                "yaw": yaw,
                "vertical": vertical,
                "fire1": self.fired1,
                "fire2": self.fired2,
                "end": end,
            }, frame
        
            




        # elif self.step == 1:
        #     self.aligned = True
        #     forward = 0
        #     lateral = 0

            
        #     if H is None:
        #         # skip (maybe go forward a bit)
        #         return {}, frame

        #     center = self.get_center(H, self.reference_image.shape, norm=True)

        #     cent = self.get_center(H, self.reference_image.shape)

        #     # Draw a visual target
        #     if self.on_top == "open":
        #         if self.fired1:
        #             self.offset_center = 0.8
        #         elif not self.fired1:
        #             self.offset_center = -0.8
        #     if self.on_top == "closed":
        #         if self.fired1:
        #             self.offset_center = -0.8
        #         elif not self.fired1:
        #             self.offset_center = 0.8

        #     cv2.circle(
        #         frame,
        #         center=(target[0], target[1]),
        #         radius=1,
        #         color=(0, 0, 255),
        #         thickness=3,
        #     )

        #     yaw, dist = self.get_orientation(H, self.reference_image.shape)

        #     if not yaw:
        #         print("[WARN] No yaw")
            
        #     # Define the offset for the different targets
        #     if (
        #         self.fired1
        #         and self.on_top == "open"
        #         or not self.fired1
        #         and self.on_top == "closed"
        #     ):
        #         center[1] += self.offset_center

        #     elif (
        #         self.fired1
        #         and self.on_top == "closed"
        #         or not self.fired1
        #         and self.on_top == "open"
        #     ):
        #         center[1] -= self.offset_center

        #     lateral = np.clip(target[0] * 3.5, -1, 1)
        #     yaw = np.clip(yaw * 3.5, -1, 1)
        #     vertical = np.clip(target[1] * 3.5, -0.2, 0.2)
        #     if abs(lateral) > self.yaw_threshold or abs(yaw) > self.yaw_threshold or abs(vertical) > self.yaw_threshold:
        #         self.aligned = False

        #     # if -self.threshold <= yaw <= self.threshold:
        #     #     # Aligned enough that we don't need to yaw
        #     #     # We can go forward now
        #     #     yaw = 0
        #     # elif yaw > self.threshold:
        #     #     print("[INFO] Yaw Left")
        #     #     yaw = 1
        #     #     self.aligned = False

        #     # if yaw < -self.threshold:
        #     #     print("[INFO] Yaw Right")
        #     #     yaw = -1
        #     #     self.aligned = False

        #     print(f"[INFO] Centers: {center}")
        #     print(f"[INFO] Dist: {dist}")

        #     cv2.circle(
        #         frame,
        #         center=(target[0], target[1]),
        #         radius=1,
        #         color=(0, 0, 255),
        #         thickness=3,
        #     )
        #     cv2.circle(
        #         frame,
        #         center=(320, 240),
        #         radius=1,
        #         color=(0, 255, 0),
        #         thickness=3,
        #     )


        #     # if center is not None:
        #     #     if target[0] > 0 + self.x_threshold:
        #     #         # Strafe Right
        #     #         print("[INFO] Right")
        #     #         lateral = 1
        #     #         self.aligned = False

        #     #     elif target[0] < 0 - self.x_threshold:
        #     #         # Strafe Left
        #     #         print("[INFO] Left")
        #     #         lateral = -1
        #     #         self.aligned = False

        #     #     if target[1] > 0 + self.y_threshold:
        #     #         # Dive
        #     #         print("[INFO] Dive")
        #     #         self.depth += 0.02
        #     #         self.aligned = False

        #     #     elif target[1] < 0 - self.y_threshold:
        #     #         # Ascent
        #     #         print("[INFO] Ascend")
        #     #         self.depth -= 0.02
        #     #         self.aligned = False

        #     if self.aligned:
        #         # Check distance from object
        #         # Now can fire or move forward

        #         if dist > self.firing_range:
        #             # Too far from target
        #             print("[INFO] Aligned, moving forward")
        #             forward = 1

        #         elif dist >= self.firing_range:
        #             # Fire
        #             if not self.fired1:
        #                 print("[INFO] Fire torpedo 1")
        #                 self.fired1 = True

        #             elif self.fired1 and self.counter > 100:
        #                 print("[INFO] Fire torpedo 2")
        #                 self.fired2 = True
        #                 end = True
        #                 print("[INFO] Mission complete")

        #         if self.fired1 and self.counter <= 100:
        #             print("[INFO] Backing up to align with closed target")
        #             forward = -1
        #             self.counter += 1
        #             print("[INFO] Counter: {counter}")



if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.torpedo_cv"

    # Create a CV object with arguments
    cv = CV()

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("testing_data/Torpedo4.mp4")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        #time.sleep(0.02)
        # run the cv
        result, img_viz = cv.run(frame, None, None)
        # print(f"[INFO] {result}")

        # show the frame
        if img_viz is not None:
            cv2.imshow("viz", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
