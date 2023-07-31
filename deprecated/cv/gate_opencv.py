"""
Description: CV template class
Author: Team Inspiration
"""

# import what you need from within the package

import time
import os

import cv2
import numpy as np
from sklearn import cluster

file_dir = os.path.dirname(os.path.abspath(__file__))

class CV:
    """Template CV class, don't change the name of the class"""

    camera = "/auv/camera/videoUSBRaw0"

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """
        self.config = config

        print(file_dir)

        self.reference_abydos = cv2.imread(f"{file_dir}/samples/abydos.png")
        self.reference_earth = cv2.imread(f"{file_dir}/samples/earth.png")
        assert self.reference_abydos is not None, "abydos.png not found"
        assert self.reference_earth is not None, "earth.png not found"

        self.reference_abydos = cv2.cvtColor(self.reference_abydos, cv2.COLOR_BGR2GRAY)
        self.reference_earth = cv2.cvtColor(self.reference_earth, cv2.COLOR_BGR2GRAY)

        cv2.imshow("abydos", self.reference_abydos)
        cv2.imshow("earth", self.reference_earth)

        self.sift = cv2.SIFT_create(contrastThreshold=0.04, edgeThreshold=10, sigma=1.6)
        self.bf = cv2.BFMatcher()

        self.abydos_kp, self.abydos_des = self.sift.detectAndCompute(self.reference_abydos, None)
        self.earth_kp, self.earth_des = self.sift.detectAndCompute(self.reference_earth, None)

        self.buffer_size = config.get("buffer_size", 20)
        self.memory_box = []
        print("[INFO] Template CV init")

    
    def process_sift(
        self,
        ref_img,
        img,
        kp1,
        des1,
        kp2=None,
        des2=None,
        threshold=0.9,
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


    def gate_box_clustering(self, frame):
        """
        Make clustering for the gate boxes, output the most probable three boxes
        """

        # if there is not enough box in memory, return None
        if len(self.memory_box) < 3:
            return None, None, None

        memory_box = np.array(self.memory_box) / 640
        # keep only the x and h values
        memory_box_trimmed = memory_box[:, [0, 3]]

        # optic clustering
        k_means = cluster.OPTICS(min_samples=3, max_eps=0.5, xi=0.5, min_cluster_size=0.1)
        k_means.fit(memory_box)

        # get the labels
        labels = k_means.labels_

        # get the number of clusters
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        # sort the boxes by x value
        box_clusters = [memory_box[labels == i].mean(axis=0) for i in range(n_clusters_)]
        box_clusters.sort(key=lambda x: x[0])

        # remove boxes that are too close to each other
        to_remove = []
        for i in range(len(box_clusters) - 1):
            if box_clusters[i + 1][0] - box_clusters[i][0] < 0.1:
                to_remove.append(i + 1)

        box_clusters = [box for i, box in enumerate(box_clusters) if i not in to_remove]

        if len(box_clusters) == 0:
            return None, None, None

        if len(box_clusters) == 1:
            box = box_clusters[0]
            if box[0] < 0.3:
                return box, None, None
            elif box[0] > 0.7:
                return None, None, box
            else:
                return None, box, None

        if len(box_clusters) == 2:
            box1 = box_clusters[0]
            box2 = box_clusters[1]

            height_diff_ratio = abs(box1[3] - box2[3]) / max(box1[3], box2[3])
            # middle gate is the smallest one
            if height_diff_ratio > 0.2:
                if box1[3] < box2[3]:
                    return None, box1, box2
                else:
                    return box1, box2, None

            left = None
            middle = None
            right = None
            if box1[0] < 0.3:
                left = box1
                if box2[0] < 0.7:
                    middle = box2
                else:
                    right = box2

            elif box1[0] > 0.3:
                middle = box1
                right = box2

            return left, middle, right

        if len(box_clusters) == 3:
            return box_clusters[0], box_clusters[1], box_clusters[2]

        return None, None, None

    def gate_on_border(self, gate_box, border=50, shape=(640, 480)):
        """
        Check if the gate is on the border of the frame
        """
        if gate_box is None:
            return False

        x, y, w, h = gate_box
        if x < border or x + w > shape[0] - border:
            return True
        return False

    def run(self, frame, target, oakd_data):
        """
        CV gate detection based on bounding box detection and ratio
        """

        # crop top of the frame
        frame = frame[200:480, 0:640]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        left_H = self.process_sift(
            self.reference_abydos,
            gray,
            self.abydos_kp,
            self.abydos_des,
            window_viz="left",
        )

        right_H = self.process_sift(
            self.reference_earth,
            gray,
            self.earth_kp,
            self.earth_des,
            window_viz="right",
        )


        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # adaptative thresholding
        edges = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

        # Find contours
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # go through all contours and find the ones with the right ratio
        for contour in contours:
            # get the bounding box of the contour as well as angle
            (x, y, w, h) = cv2.boundingRect(contour)

            # get the ratio of the bounding box
            ratio = w / h
            area = w * h

            # if the ratio is good, draw the bounding box
            if ratio > 0.0 and ratio < 0.3 and area > 1000:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 5)
                self.memory_box.append((x, y, w, h))
                if len(self.memory_box) > self.buffer_size:
                    self.memory_box.pop(0)

        left_gate, middle_gate, right_gate = self.gate_box_clustering(frame)

        if left_gate is not None:
            left_gate = (left_gate * 640).astype(int)
            (x, y, w, h) = left_gate
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 5)
        if middle_gate is not None:
            middle_gate = (middle_gate * 640).astype(int)
            (x, y, w, h) = middle_gate
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5)
        if right_gate is not None:
            right_gate = (right_gate * 640).astype(int)
            (x, y, w, h) = right_gate
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 5)

        # detect glyphs

        if left_gate is not None and right_gate is not None:
            # both legs are on the border = detect abydos (TODO)
            if self.gate_on_border(left_gate) and self.gate_on_border(right_gate):
                return {"lateral": 0, "forward": 0, "end": True}, frame

            # both legs are not on the border = go forward
            if not self.gate_on_border(left_gate) and not self.gate_on_border(right_gate):
                return {"lateral": 0, "forward": 0.5, "end": False}, frame

        cv2.imshow("edges", edges)

        return {"lateral": 0, "forward": 0, "end": False}, frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes

    # Create a CV object with arguments
    cv = CV()

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("testing_data\\gate.mkv")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        # run the cv
        result, frame = cv.run(frame, None, None)
        print(f"[INFO] {result}")

        # debug the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(33) & 0xFF == ord("q"):
            break
