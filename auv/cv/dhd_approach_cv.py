"""
CV script for approaching the DHD platform.
"""

import cv2
import numpy as np
from circle_fit import taubinSVD


class CV:
    """
    CV class for approaching the DHD platform.
    """

    def __init__(self, **config):
        """
        Initialize the DHD platform CV class.

        Args:
            config (dict): Contains the configuration of the devices on the sub.
        """

        self.config = config
        self.current_sub = self.config.get("sub", "graey")

        # Camera to get the camera feed from, ML model to run.
        self.camera = "/auv/camera/videoOAKdRawForward"
        self.model = "dhdA"

        self.viz_frame = None
        self.error_buffer = []

        self.prev_pos = None

        print("[INFO] Surfacing CV init")

    def get_error(self, center_x, center_y, shape=(480, 640)):
        """
        Calculates the error of the center point of the target from the center of the frame, normalized to the frame size.

        Args:
            center_x: Target's x-coordinate on the screen.
            center_y: Target's y-coordinate on the screen.
            shape (tuple): Size of the frame, defaults to (480, 640) -> (height, width)

        Returns:
            tuple: x-error, y-error
        """
        x_error = (center_x - shape[1] / 2) / shape[1]
        y_error = (shape[0] - center_y) / shape[0]
        return (x_error, y_error)

    def run(self, frame, target, detections):
        """
        Run the DHD Approach CV script. Uses the detections from the ML model to approach the octagon/DHD.

        Args:
            frame: Frame from the camera feed.
            target: Target to aim for.
            detections: The list of detections with the relevant data outputted by the ML model.

        Returns:
            dictionary, numpy.ndarray: {lateral movement command, forward movement command, yaw command, flag of state (ended or not ended)}, visualized frame.
        """

        self.viz_frame = frame

        # If there are no detections, that means that either move forward to find a detection, or we are close to the DHD, so we can end the mission.
        if detections is None or len(detections) == 0:
            if self.prev_pos is None or self.prev_pos[1] < 440:
                return {"lateral": 0, "forward": 1.5, "yaw": 0}, self.viz_frame
            else:
                # We are close to DHD, end the mission.
                return {"lateral": 0, "forward": 0, "yaw": 0, "end": True}, self.viz_frame

        target = None
        confidence = 0
        ymin = 480

        # Find the detection with the greatest labeled confidence. Make that one the target.
        for detection in detections:
            if detection.confidence > confidence:
                target = detection
                confidence = detection.confidence
                ymin = detection.ymin
        
        # Compute the target's center. 
        target_center = int((target.xmin + target.xmax) / 2), int((target.ymin + target.ymax) / 2)
        if target_center[1] > 450:
            return {"end": True}, self.viz_frame

        cv2.circle(self.viz_frame, target_center, 5, (0, 0, 255), 2)

        x_error, y_error = self.get_error(target_center[0], target_center[1])

        # Calculate the yaw and forward commands.
        yaw = np.clip(x_error * 3.5, -1.0, 1.0)
        forward = np.clip(y_error * 10, 1, 2.5)

        self.prev_pos = target_center
        return {"lateral": 0, "forward": forward, "yaw": yaw}, self.viz_frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly.
    # It is here for testing purposes.
    # you can run this file independently using: "python -m auv.cv.dhd_approach_cv".

    # Create a CV object with arguments.
    cv = CV()

    # Capture video for training data purposes.
    cap = cv2.VideoCapture("testing_data\\dhd_approach.mp4")

    while True:
        ret, img = cap.read()
        if not ret:
            break

        # Run the CV script.
        result, img_viz = cv.run(img, None, None)
        print(f"[INFO] {result}")

        # Show the result on the screen (the visualized frame).
        cv2.imshow("frame", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
