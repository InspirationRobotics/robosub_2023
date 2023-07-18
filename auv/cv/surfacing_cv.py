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

    def __init__(self, **config):
        """
        Init of surfacing CV
        """
        self.config = config
        self.current_sub = self.config.get("sub", "greay")
        if self.current_sub == "greay" :
            self.camera = "/auv/camera/videoUSBRaw1"
        elif self.current_sub == "onyx" :
            self.camera = "/auv/camera/videoOAKdRawBottom"

        self.surfacing_sensitivity = config.get("surfacing_sensitivity", 3.0)

        self.viz_frame = None
        self.error_buffer = []

        logger.info("Surfacing CV init")

    def get_octogon_center(self, frame):
        """
        Returns the center of the octogon in the frame
        using a contours detection approach
        """

        # equalize channels
        frame_b, frame_g, frame_r = cv2.split(frame)
        frame_g = cv2.equalizeHist(frame_g)
        frame_b = cv2.equalizeHist(frame_b)
        frame_r = cv2.equalizeHist(frame_r)
        frame = cv2.merge((frame_b, frame_g, frame_r))

        # filter the image to red objects, filters what is white
        gray = cv2.inRange(frame, (0, 0, 200), (10, 10, 255))

        # get the centroid of the red points
        M = cv2.moments(gray)
        if M["m00"] == 0:
            return None, None
        x_center = int(M["m10"] / M["m00"])
        y_center = int(M["m01"] / M["m00"])

        # get standard deviation of the red points
        std = np.std(gray)

        cv2.circle(self.viz_frame, (x_center, y_center), 10, (0, 0, 255), -1)

        return x_center, y_center

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

        (x_center, y_center) = self.get_octogon_center(frame)

        if x_center is None or y_center is None:
            return {}, self.viz_frame

        (x_error, y_error) = self.get_error(x_center, y_center, frame.shape)
        self.error_buffer.append((x_error, y_error))
        if len(self.error_buffer) > 30:
            self.error_buffer.pop(0)

        avg_error = np.mean(np.linalg.norm(self.error_buffer, axis=1))
        if avg_error < 0.05 and len(self.error_buffer) == 30:
            return {"lateral": 0, "forward": 0, "end": True}, self.viz_frame

        # TODO: implement a simple PID controller for everyone to use
        x_error *= self.surfacing_sensitivity
        y_error *= self.surfacing_sensitivity

        # if greay, x = forward, y = lateral
        if self.current_sub == "greay" :
            return {"lateral": y_error, "forward": x_error, "end": False}, self.viz_frame
        else:
            return {"lateral": x_error, "forward": y_error, "end": False}, self.viz_frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.surfacing_cv"
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
