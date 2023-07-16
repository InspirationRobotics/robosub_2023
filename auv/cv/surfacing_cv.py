"""
Description: CV class for surfacing
Author: Maxime Ellerbach
"""

import logging

import cv2
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class CV:
    camera = "/auv/camera/videoUSBRaw1"

    def __init__(self, **config):
        """
        Init of surfacing CV
        """

        self.viz_frame = None
        self.memory_edges = None
        self.error_buffer = []

        self.surfacing_sensitivity = config.get("surfacing_sensitivity", 2.0)

        logger.info("Surfacing CV init")

    def get_octogon_center(self):
        """
        Returns the center of the octogon in the frame
        using a contours detection approach
        """

        # convert to grayscale
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # apply Canny edge detection
        edges = cv2.Canny(image=gray, threshold1=0, threshold2=200)
        edges = cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations=2)

        # combine with memory edges
        self.memory_edges = cv2.addWeighted(self.memory_edges, 0.85, edges, 0.15, 0)
        _, edges = cv2.threshold(self.memory_edges, 60, 255, cv2.THRESH_BINARY)

        # find contours, convex hull, and approx poly
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        contours = [cv2.convexHull(contour) for contour in contours]
        contours = [contour for contour in contours if cv2.contourArea(contour) > 1000]

        if contours is None or len(contours) == 0:
            return (None, None)

        # find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # draw contours
        self.frame = cv2.drawContours(self.frame, contours, -1, (0, 255, 0), 2)
        self.frame = cv2.drawContours(self.frame, [largest_contour], -1, (0, 255, 255), 2)

        # find the center of the contour
        M = cv2.moments(largest_contour)

        if M["m00"] == 0:
            return (None, None)

        x_center = int(M["m10"] / M["m00"])
        y_center = int(M["m01"] / M["m00"])
        self.frame = cv2.circle(self.frame, (x_center, y_center), 5, (0, 0, 255), -1)
        return (x_center, y_center)

    def get_error(self, center_x, center_y, shape):
        """Returns the error in x and y, normalized to the frame size."""
        max_size = max(shape[0], shape[1]) / 2
        x_error = (shape[1] / 2 - center_x) / max_size
        y_error = (shape[0] / 2 - center_y) / max_size
        return (x_error, y_error)

    def run(self, frame, target, oakd_data):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        self.viz_frame = frame
        if self.memory_edges is None:
            self.memory_edges = np.zeros((frame.shape[0], frame.shape[1], 1), dtype=np.uint8)

        (x_center, y_center) = self.get_octogon_center()

        if x_center is None or y_center is None:
            return {}, self.viz_frame

        (x_error, y_error) = self.get_error(x_center, y_center, frame.shape)
        self.error_buffer.append((x_error, y_error))
        if len(self.error_buffer) > 20:
            self.error_buffer.pop(0)

        avg_error = np.mean(np.linalg.norm(self.error_buffer, axis=1))
        if avg_error < 0.05:
            return {"lateral": 0, "forward": 0, "end": True}, self.viz_frame

        x_error *= self.surfacing_sensitivity
        y_error *= self.surfacing_sensitivity
        return {"lateral": x_error, "forward": y_error, "end": False}, self.viz_frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template"
    logging.basicConfig(level=logging.INFO)

    # Create a CV object with arguments
    cv = CV()

    cap = cv2.VideoCapture("testing_data\\octogon.mp4")

    while True:
        ret, img = cap.read()
        if not ret:
            break

        # set the frame
        img = cv2.resize(img, (480, 640))

        # run the CV
        result, img_viz = cv.run(img, None, None)
        logger.info(result)

        # show the result
        cv2.imshow("frame", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
