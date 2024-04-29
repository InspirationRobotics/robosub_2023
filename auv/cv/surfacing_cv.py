"""
CV script for the surfacing mission.
"""

import time
import cv2
import numpy as np
from circle_fit import taubinSVD


class CV:
    """
    CV class for the surfacing mission.
    """
    def __init__(self, **config):
        """
        Initialize the CV class.

        Args:
            config: A dictionary containing the configuration of the devices on a sub.
        """
        self.config = config
        self.current_sub = self.config.get("sub", "graey")

        # Use the specified camera and ML model based on the sub in use.
        if self.current_sub == "graey":
            self.camera = "/auv/camera/videoUSBRaw0" # Camera to get camera stream from.
            self.model = "raw" # ML model to run (in this case none, or "raw", since only OAK-Ds can run models).
            self.run = self.run_graey
        elif self.current_sub == "onyx":
            self.camera = "/auv/camera/videoOAKdRawBottom" # Camera to get camera stream from.
            self.model = "dhd" # ML model to run.
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
        """
        Equalize the histogram of the frame.

        Args:
            frame (numpy.ndarray): Color frame from the camera (comes in BGR color format).

        Returns:
            frame (numpy.ndarray): Equalized frame.
        """
        frame_b, frame_g, frame_r = cv2.split(frame)
        frame_g = cv2.equalizeHist(frame_g)
        frame_b = cv2.equalizeHist(frame_b)
        frame_r = cv2.equalizeHist(frame_r)
        frame = cv2.merge((frame_b, frame_g, frame_r))
        return frame

    def get_octogon_center_color(self, frame, viz=False):
        """
        Get the center of the octagon in the frame, using a color detection method.
        # NOTE: This is a very naive approach and it will not work in all conditions.

        Args:
            frame (numpy.ndarray): Frame from the camera feed (equalized or nonequalized) 
            viz (bool): Whether or not to show the visualized frame.

        Returns:
            tuple: X-coordinate of the octagon center, y-coordinate of the octagon center.
        """

        # Filter the white pixels.
        gray = cv2.inRange(frame, (200, 200, 200), (255, 255, 255))

        # if viz:
        # self.viz_frame = np.concatenate((self.viz_frame, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)), axis=1)

        # Find the largest contour; this is the actual octagon.
        contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None, None
        contour = max(contours, key=cv2.contourArea)

        # Find the centroid of the large shape (octagon) using moments.
        M = cv2.moments(contour)

        if M["m00"] == 0 or cv2.contourArea(contour) < 20000:
            return None, None
        x_center = int(M["m10"] / M["m00"])
        y_center = int(M["m01"] / M["m00"])

        # Draw a circle around the center of the octagon.
        cv2.circle(self.viz_frame, (x_center, y_center), 10, (0, 255, 0), -1)

        return x_center, y_center

    def get_octogon_center_model(self, detections, th=0.5):
        """
        Get the center of the octagon using the output from the ML model.

        Args:
            detections: List of detections outputted by the ML model.
            th (float): Threshold confidence.

        Returns:
            tuple: X-coordinate of the center, y-coordinate of the center.
        """
        for detection in detections:
            if detection.label == "DHD" and detection.confidence > th:
                return (detection.xmin + detection.xmax) / 2, (detection.ymin + detection.ymax) / 2

        symbols = {}
        
        # Ensure there are no duplicate symbols.
        for detection in detections:
            label = detection.label
            # If there are two detections that have the same label, only take the one that has the higher confidence.
            if label in symbols and detection.confidence > symbols[label].confidence:
                symbols[label] = detection
            elif detection.label not in symbols:
                symbols[label] = detection

        if len(symbols) < 3:
            return None, None

        # Get the coordinates of each of the symbols.
        coords = [(d.xmin + d.xmax) / 2 + (d.ymin + d.ymax) / 2 for d in symbols.values()]

        # Find the center of the octagon based on calculating the center point in between the symbols.
        xc, yc, r, sigma = taubinSVD(coords)

        cv2.circle(self.viz_frame, (int(xc), int(yc)), int(r), (0, 0, 255), 2)
        return xc, yc

    def get_error(self, center_x, center_y, shape=(480, 640)):
        """
        Return the x and y error, normalized to the size of the screen.

        Args:
            center_x: Center x-coordinate of the target.
            center_y: Center y-coordinate of the target.
            Shape (tuple): Shape of the frame (height, width), defaults to (480, 640).
        """
        max_size = max(shape[0], shape[1]) / 2
        x_error = (center_x - shape[1] / 2) / max_size
        y_error = (shape[0] / 2 - center_y) / max_size
        return (x_error, y_error)

    def run_graey(self, frame, target, detections):
        """
        Run the surfacing CV for Graey.

        Args:
            frame: Frame from the camera.
            target: Target of the mission.
            Detections: List of detections from the ML model.

        Returns:
            dictionary, numpy.ndarray: {lateral movement command, forward movment command, end flag}, visualized frame.
        """
        # frame = self.equalize(frame)
        self.viz_frame = frame

        (x_center, y_center) = self.get_octogon_center_color(frame, viz=True)

        # Move slowly forward if the octagon is not found.
        if x_center is None or y_center is None:
            return {"lateral": 0, "forward": 1.5, "end": False}, self.viz_frame

        # Get the error of the target coordinates in the frame relative to the center of the frame.
        (x_error, y_error) = self.get_error(x_center, y_center)
        self.error_buffer.append((x_error, y_error))

        # Keep only the 30 most updated error values.
        if len(self.error_buffer) > 30:
            self.error_buffer.pop(0)
            if self.start_timeout is None:
                self.start_timeout = time.time()

        # Average the error; if the error is low enough and the error is accurate (there are enough values in the error buffer that were averaged), then we have completed 
        # the mission.
        avg_error = np.mean(np.linalg.norm(self.error_buffer, axis=1))
        if avg_error < 0.2 and len(self.error_buffer) == 30:
            return {"lateral": 0, "forward": 0, "end": True}, self.viz_frame

        # If the time alloted for running the surfacing mission is over, then end the mission.
        if self.start_timeout and self.start_timeout + self.timeout < time.time():
            print("surfacing timeout")
            return {"lateral": 0, "forward": 0, "end": True}, self.viz_frame

        # Calculate the lateral and forward values by applying gain and clipping.
        lateral = np.clip(x_error * 3, -1, 1)
        forward = np.clip(y_error * 3, -1, 1)

        return {"lateral": lateral, "forward": forward, "end": False}, self.viz_frame

    def run_onyx(self, frame, target, detections):
        """
        Run the surfacing CV for Onyx.

        Args:
            frame: Frame from the camera.
            target: Target of the mission.
            Detections: List of detections from the ML model.

        Returns:
            dictionary, numpy.ndarray: {lateral movement command, forward movment command, end flag}, visualized frame.
        """
        self.viz_frame = frame

        # Get the center of the octagon.
        (x_center, y_center) = self.get_octogon_center_model(detections)

        # Move slowly forward if the octagon is not found.
        if x_center is None or y_center is None:
            return {"lateral": 0, "forward": 1.5, "end": False}, self.viz_frame

        # Get the error of the target coordinates on the frame relative to the center of the frame.
        (x_error, y_error) = self.get_error(x_center, y_center)

        # Keep track of the errors, and only keep the 30 most updated values.
        self.error_buffer.append((x_error, y_error))
        if len(self.error_buffer) > 30:
            self.error_buffer.pop(0)

        # If the error is low enough, and there are enough errors that the average is accurate, then end the mission.
        avg_error = np.mean(np.linalg.norm(self.error_buffer, axis=1))
        if avg_error < 0.15 and len(self.error_buffer) == 30:
            return {"lateral": 0, "forward": 0, "end": True}, self.viz_frame

        # Calculate the forward and lateral movement commands by applying a gain and clipping the values.
        lateral = np.clip(x_error * 3.5, -1, 1)
        forward = np.clip(y_error * 3.5, -1, 1)

        return {"lateral": lateral, "forward": forward, "end": False}, self.viz_frame

    # self.run will be set to the correct function in __init__ depending on the sub


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly.
    # It is here for testing purposes.
    # you can run this file independently using: "python -m auv.cv.surfacing_cv".

    # Create a CV object with arguments.
    cv = CV()

    cap = cv2.VideoCapture("testing_data\\octagon_bottom_graey2.mp4")

    while True:
        ret, img = cap.read()
        if not ret:
            break

        # Resize the frame
        # img = cv2.resize(img, (480, 640))

        # Run the CV script.
        result, img_viz = cv.run(img, None, None)
        print(f"[INFO] {result}")

        # Show the result.
        cv2.imshow("frame", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
