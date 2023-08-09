"""
Description: CV class for surfacing
Author: Maxime Ellerbach
"""

import time
import cv2
import numpy as np
from circle_fit import taubinSVD


class CV:
    def __init__(self, **config):
        """
        Init of surfacing CV
        """
        self.config = config
        self.current_sub = self.config.get("sub", "graey")
        if self.current_sub == "graey":
            self.camera = "/auv/camera/videoUSBRaw0"
            self.model = "raw"
            self.run = self.run_graey
        elif self.current_sub == "onyx":
            self.camera = "/auv/camera/videoOAKdRawBottom"
            self.model = "dhd"
            self.run = self.run_onyx
        else:
            self.run = lambda frame, target, detections: None
            print(f"[ERROR] Unknown sub {self.current_sub} in [graey, onyx]")
            raise Exception(f"Unknown sub {self.current_sub}")

        self.viz_frame = None
        self.error_buffer = []
        self.timeout = 30
        self.start_timeout = None

        print("[INFO] Surfacing CV init")

    def equalize(self, frame):
        frame_b, frame_g, frame_r = cv2.split(frame)
        frame_g = cv2.equalizeHist(frame_g)
        frame_b = cv2.equalizeHist(frame_b)
        frame_r = cv2.equalizeHist(frame_r)
        frame = cv2.merge((frame_b, frame_g, frame_r))
        return frame

    def get_octogon_center_color(self, frame, viz=False):
        """
        Returns the center of the octogon in the frame
        using a color detection approach
        # NOTE: this is a very naive approach, it will not work in all conditions, will fix later
        """
        # filters what is white
        gray = cv2.inRange(frame, (200, 200, 200), (255, 255, 255))
        # if viz:
        # self.viz_frame = np.concatenate((self.viz_frame, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)), axis=1)

        # get biggest contour
        contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None, None
        contour = max(contours, key=cv2.contourArea)

        # get the centroid of the white points
        M = cv2.moments(contour)

        if M["m00"] == 0 or cv2.contourArea(contour) < 20000:
            return None, None
        x_center = int(M["m10"] / M["m00"])
        y_center = int(M["m01"] / M["m00"])

        # get standard deviation of the red points
        cv2.circle(self.viz_frame, (x_center, y_center), 10, (0, 255, 0), -1)

        return x_center, y_center

    def get_octogon_center_model(self, detections, th=0.5):
        for detection in detections:
            if detection.label == "DHD" and detection.confidence > th:
                return (detection.xmin + detection.xmax) / 2, (detection.ymin + detection.ymax) / 2

        symbols = {}
        # ensure no duplicate symbols
        for detection in detections:
            label = detection.label
            if label in symbols and detection.confidence > symbols[label].confidence:
                symbols[label] = detection
            elif detection.label not in symbols:
                symbols[label] = detection

        if len(symbols) < 3:
            return None, None

        # get coordinates of each symbols
        coords = [(d.xmin + d.xmax) / 2 + (d.ymin + d.ymax) / 2 for d in symbols.values()]
        xc, yc, r, sigma = taubinSVD(coords)

        cv2.circle(self.viz_frame, (int(xc), int(yc)), int(r), (0, 0, 255), 2)
        return xc, yc

    def get_error(self, center_x, center_y, shape=(480, 640)):
        """Returns the error in x and y, normalized to the frame size."""
        max_size = max(shape[0], shape[1]) / 2
        x_error = (center_x - shape[1] / 2) / max_size
        y_error = (shape[0] / 2 - center_y) / max_size
        return (x_error, y_error)

    def run_graey(self, frame, target, detections):
        # frame = self.equalize(frame)
        self.viz_frame = frame

        (x_center, y_center) = self.get_octogon_center_color(frame, viz=True)

        # move slowly forward if the octogon is not found
        if x_center is None or y_center is None:
            return {"lateral": 0, "forward": 1.5, "end": False}, self.viz_frame

        (x_error, y_error) = self.get_error(x_center, y_center)
        self.error_buffer.append((x_error, y_error))
        if len(self.error_buffer) > 30:
            self.error_buffer.pop(0)
            if self.start_timeout is None:
                self.start_timeout = time.time()

        avg_error = np.mean(np.linalg.norm(self.error_buffer, axis=1))
        if avg_error < 0.2 and len(self.error_buffer) == 30:
            return {"lateral": 0, "forward": 0, "end": True}, self.viz_frame

        if self.start_timeout and self.start_timeout + self.timeout < time.time():
            print("surfacing timeout")
            return {"lateral": 0, "forward": 0, "end": True}, self.viz_frame

        # apply a gain and clip the values
        lateral = np.clip(x_error * 3, -1, 1)
        forward = np.clip(y_error * 3, -1, 1)

        return {"lateral": lateral, "forward": forward, "end": False}, self.viz_frame

    def run_onyx(self, frame, target, detections):
        self.viz_frame = frame

        (x_center, y_center) = self.get_octogon_center_model(detections)

        # move slowly forward if the octogon is not found
        if x_center is None or y_center is None:
            return {"lateral": 0, "forward": 1.5, "end": False}, self.viz_frame

        (x_error, y_error) = self.get_error(x_center, y_center)
        self.error_buffer.append((x_error, y_error))
        if len(self.error_buffer) > 30:
            self.error_buffer.pop(0)

        avg_error = np.mean(np.linalg.norm(self.error_buffer, axis=1))
        if avg_error < 0.15 and len(self.error_buffer) == 30:
            return {"lateral": 0, "forward": 0, "end": True}, self.viz_frame

        # apply a gain and clip the values
        lateral = np.clip(x_error * 3.5, -1, 1)
        forward = np.clip(y_error * 3.5, -1, 1)

        return {"lateral": lateral, "forward": forward, "end": False}, self.viz_frame

    # self.run will be set to the correct function in __init__ depending on the sub


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.surfacing_cv"

    # Create a CV object with arguments
    cv = CV()

    cap = cv2.VideoCapture("testing_data\\octagon_bottom_graey2.mp4")

    while True:
        ret, img = cap.read()
        if not ret:
            break

        # resize the frame
        # img = cv2.resize(img, (480, 640))

        # run the CV
        result, img_viz = cv.run(img, None, None)
        print(f"[INFO] {result}")

        # show the result
        cv2.imshow("frame", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
