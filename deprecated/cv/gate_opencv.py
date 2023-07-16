"""
Description: CV template class
Author: Team Inspiration
"""

# import what you need from within the package

import logging
import time

import cv2
import numpy as np
from sklearn import cluster

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class CV:
    """Template CV class, don't change the name of the class"""

    camera = "/auv/camera/videoUSBRaw0"

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """

        self.buffer_size = config.get("buffer_size", 30)
        self.memory_box = []
        logger.info("Template CV init")

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

        # # viz the clusters on the frame
        # for i in range(n_clusters_):
        #     # get the cluster
        #     (x, y, w, h) = (memory_box[labels == i].mean(axis=0) * 640).astype(int)
        #     standard_deviation = memory_box_trimmed[labels == i].std(axis=0)

        #     # draw the bounding box
        #     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 5)

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



    def run(self, frame, target, oakd_data):
        """
        CV gate detection based on bounding box detection and ratio
        """

        # crop top of the frame
        frame = frame[200:480, 0:640]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        # adaptative thresholding
        edges = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

        # # Apply a morphological operation to close gaps between edge segments
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

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
        left_gate = (left_gate * 640).astype(int) if left_gate is not None else None
        middle_gate = (middle_gate * 640).astype(int) if middle_gate is not None else None
        right_gate = (right_gate * 640).astype(int) if right_gate is not None else None

        if left_gate is not None:
            (x, y, w, h) = left_gate
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 5)
        if middle_gate is not None:
            (x, y, w, h) = middle_gate
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5)
        if right_gate is not None:
            (x, y, w, h) = right_gate
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 5)

        cv2.imshow("edges", edges)

        return {"lateral": 0, "forward": 0, "end": False}, frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    logging.basicConfig(level=logging.INFO)

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
        logger.info(result)

        # debug the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(33) & 0xFF == ord("q"):
            break
