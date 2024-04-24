"""
CV script for the bins mission.
Drops the markers into the correct side of the bin
"""

import time

import cv2
import numpy as np
import shapely


class CV:
    """
    Class for the Bins CV script
    """

    def __init__(self, **config):
        """
        Initialize the Bins CV class.

        Args:
            config: Mission specific parameters for running the mission
        """
        self.config = config
        self.current_sub = self.config.get("sub", "onyx")

        # If the sub is Onyx, get the correct camera and ML model
        if self.current_sub == "onyx":
            self.camera = "/auv/camera/videoOAKdRawBottom"
            self.model = "bins3"
        elif self.current_sub == "graey":
            print(f"[INFO] No Gripper or Dropper on graey")
            self.camera = None

        self.viz_frame = None
        self.error_buffer = []

        print("[INFO] Bin CV init")

    def mask_out_lids(self, target, lids):
        """
        Masks out the lids. Masking out means removing the lids from the image.
        This method essentially detects each lid and removes it from the image, so that all that is left is the actual bin.

        Args:
            target: The target bin
            lids: The list of lids (from the list of detections present outputted by the ML model)
        
        Returns:
            - A new target bin with the lids masked out
            - The center of that target bin
        """

        # If there are no lids, then just return the target and the center of the target
        if len(lids) == 0:
            return target, self.get_bbox_center(target)

        # Create a box around the target bin
        x1, x2, y1, y2 = target.xmin, target.xmax, target.ymin, target.ymax
        geom_target = shapely.geometry.box(x1, y1, x2, y2)

        # Mask out the lids by subtracting the lid from the target bin's image
        for lid in lids:
            x1, x2, y1, y2 = lid.xmin - 100, lid.xmax + 100, lid.ymin, lid.ymax
            geom_lid = shapely.geometry.box(x1, y1, x2, y2)
            geom_target = geom_target.difference(geom_lid)

        # If there are multiple polygons in the target bin, pick the largest one -- this is the actual bin, not the handle of the lid or something like that
        if geom_target.geom_type == "MultiPolygon":
            geom_target = max(geom_target, key=lambda x: x.area)

        # If there is a hole in the polygon, split it into two -- this is so that if the lid is in the center of the lid, we can still process and drop the marker
        if geom_target.geom_type == "Polygon" and geom_target.interiors:
            x1, x2, y1, y2 = geom_target.interiors[0].bounds

            # Draw the horizontal line
            split_line = shapely.geometry.LineString([(0, (y1 + y2) / 2), (640, (y1 + y2) / 2)])

            # Split the polygon with the line
            splitted = shapely.ops.split(geom_target, split_line)

            # Pick the largest polygon -- this is so taht there is a higher likelihood that we are able to drop the markers into the bin
            geom_target = max(splitted, key=lambda x: x.area)

        x1, y1, x2, y2 = geom_target.bounds
        target.xmin = x1
        target.ymin = y1
        target.xmax = x2
        target.ymax = y2

        # Get the centroid of the target bin
        centroid = geom_target.centroid
        target_center = (int(centroid.x), int(centroid.y))

        return target, target_center

    def get_bbox_center(self, detection):
        """
        Get the center of a box (in this case the target bin)

        Args:
            detection: Information on the bin from the ML model
        """
        x1 = int(detection.xmin)
        x2 = int(detection.xmax)
        y1 = int(detection.ymin)
        y2 = int(detection.ymax)

        # Use midpoint formula
        return ((x1 + x2) // 2, (y1 + y2) // 2)

    def run(self, frame, target, oakd_data):
        """
        Run the Bin CV script

        Args:
            frame: The frame from the camera stream
            target: The target bin
            oakd_data: This is the list of detections

        Returns:
            dictionary, frame: {lateral motion command, forward motion command, flag representing state of alignment}, visualized frame
        """

        forward = 0
        lateral = 0
        aligned = False

        height, width, _ = frame.shape

        tolerance = 0.1
        maxConfidence = 0
        target_bin = None
        earth = None
        abydos = None
        earth_confidence = 0
        abydos_confidence = 0
        bins = []
        lids = []

        # Based on measured offset from the footage -- this is where we want the center of the target to be on the frame
        target_pixel = (190, 300)

        # If there is no data, just move forward to try and find the apparatus
        if len(oakd_data) == 0:
            return {"forward": 0.8}, frame

        # Parse through and identify each detection based on the ML model's output (the list of detections)
        for detection in oakd_data:
            # Get the coordinates of the detection
            x1 = int(detection.xmin)
            x2 = int(detection.xmax)
            y1 = int(detection.ymin)
            y2 = int(detection.ymax)

            # Identify the detection
            if "abydos" in detection.label and detection.confidence > abydos_confidence:
                abydos = detection
                abydos_confidence = detection.confidence

            elif "earth" in detection.label and detection.confidence > earth_confidence:
                earth = detection
                earth_confidence = detection.confidence

            elif "bin" in detection.label:
                bins.append(detection)

            elif "lid" in detection.label:
                lids.append(detection)

        # Choose the detection to be the target based on the target that was passed in (either "earth" or "abydos")
        if "earth" in target:
            target_bin = earth

        elif "abydos" in target:
            target_bin = abydos

        approach = False

        # If there is no target bin
        if target_bin is None:
            # If there are no bins detected
            if earth is None and abydos is None:
                # Move forward and try to find the bins if there is no bin whatsoever
                if len(bins) == 0:
                    return {"forward": 0.8}, frame
                # Else average the center of the bins and try to move forward by setting the target center to that averaged center
                # This is not ideal, this is if the ML model failed (in detecting which bin was which)
                else:
                    # Average the centers of the bins
                    avg_center = np.mean([self.get_bbox_center(b) for b in bins], axis=0)
                    target_bin_center = (int(avg_center[0]), int(avg_center[1]))
                    approach = True
            elif earth is None:
                target_bin = abydos
            elif abydos is None:
                target_bin = earth

        if not approach:
            # Remove the lids from the target bin (mask out the lids)
            target_bin, target_bin_center = self.mask_out_lids(target_bin, lids)

        # Draw a rectangle around the target bin
        cv2.rectangle(
            frame,
            (int(target_bin.xmin), int(target_bin.ymin)),
            (int(target_bin.xmax), int(target_bin.ymax)),
            (255, 0, 0),
            2,
        )

        # Draw a circle around the target pixel where the center of the target bin should be, and the actual target bin center on the frame
        cv2.circle(frame, target_pixel, 10, (0, 255, 0), -1)
        cv2.circle(frame, target_bin_center, 10, (0, 0, 255), -1)

        # Compute the normalized x and y error for the alignment with the target pixel
        x_error = (target_bin_center[0] - target_pixel[0]) / width
        y_error = (target_pixel[1] - target_bin_center[1]) / height

        # Apply a gain and clip the values for the motion commands
        lateral = np.clip(x_error * 3.5, -1, 1)
        forward = np.clip(y_error * 3.5, -1, 1)

        # Only store the 30 most updated errors -- delete the least updated one if there are more than 30
        if len(self.error_buffer) > 30:
            self.error_buffer.pop(0)

        # Average the x and y alignment error, store it in the error list
        self.error_buffer.append((x_error + y_error) / 2)

        # Average out the error across the last 30 entries
        avg_error = np.mean(np.linalg.norm(self.error_buffer, axis=1))

        # If the average error is low enough, then we are aligned
        if avg_error < tolerance and len(self.error_buffer) == 30:
            aligned = True

        # TODO (low priority): Remove lids for each bin
        return {"lateral": lateral, "forward": forward, "aligned": aligned}, frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # You can run this file independently using: "python -m auv.cv.bin_cv"

    # Create a CV object with arguments
    cv = CV()

    # Here you can initialize the camera, etc.
    cap = cv2.VideoCapture("../../testing_data/")

    while True:
        # Grab a frame from the camera stream
        ret, frame = cap.read()
        if not ret:
            break

        # Run the CV script
        result = cv.run(frame, "some_info", None)

        # Do something with the result
        print(f"[INFO] {result}")

        # Debug the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
