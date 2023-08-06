"""
Description: CV class for surfacing
Author: Maxime Ellerbach
"""

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

        self.camera = "/auv/camera/videoOAKdRawForward"
        self.model = "dhdA"

        self.viz_frame = None
        self.error_buffer = []

        self.prev_pos = None

        print("[INFO] Surfacing CV init")

    def get_error(self, center_x, center_y, shape=(480, 640)):
        """Returns the error in x and y, normalized to the frame size."""
        x_error = (center_x - shape[1] / 2) / shape[1]
        y_error = (shape[0] - center_y) / shape[0]
        return (x_error, y_error)

    def run(self, frame, target, detections):
        """Use detections approach (yaw and forward) to the octogon."""

        self.viz_frame = frame

        if detections is None or len(detections) == 0:
            if self.prev_pos is None or self.prev_pos[1] < 440:
                return {"lateral": 0, "forward": 1.5, "yaw": 0}, self.viz_frame
            else:
                # we are close to dhd, end mission
                return {"lateral": 0, "forward": 0, "yaw": 0, "end": True}, self.viz_frame

        target = None
        confidence = 0
        ymin = 480
        for detection in detections:
            if detection.confidence > confidence:
                target = detection
                confidence = detection.confidence
                ymin = detection.ymin
        target_center = int((target.xmin + target.xmax) / 2), int((target.ymin + target.ymax) / 2)
        if target_center[1] > 450:
            return {"end": True}, self.viz_frame

        cv2.circle(self.viz_frame, target_center, 5, (0, 0, 255), 2)

        x_error, y_error = self.get_error(target_center[0], target_center[1])

        yaw = np.clip(x_error * 3.5, -1.0, 1.0)
        forward = np.clip(y_error * 10, 1, 2.5)

        self.prev_pos = target_center
        return {"lateral": 0, "forward": forward, "yaw": yaw}, self.viz_frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.surfacing_cv"

    # Create a CV object with arguments
    cv = CV()

    cap = cv2.VideoCapture("testing_data\\dhd_approach.mp4")

    while True:
        ret, img = cap.read()
        if not ret:
            break

        # run the CV
        result, img_viz = cv.run(img, None, None)
        print(f"[INFO] {result}")

        # show the result
        cv2.imshow("frame", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
