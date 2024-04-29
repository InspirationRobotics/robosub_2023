"""
CV code for the Follow the Path Task
"""

import time

import cv2
import numpy as np

from ..utils import deviceHelper

class CV:
    """
    CV class for the Follow the Path task.
    """

    # Camera to get feed from.
    camera = "/auv/camera/videoUSBRaw1"

    def __init__(self, **config):
        """
        Initialize the Follow the Path CV task.

        Args:
            config: Dictionary containing the configuration of the devices on the sub
        """
        self.config = config
        self.config = deviceHelper.variables
        self.aligned = False
        self.state = "strafe"
        self.current_sub = self.config.get("sub", "graey")

        # Use low-light camera #1 for Graey (0 is forward, 1 is bottom-facing), or OAK-D bottom-facing camera for Onyx
        if self.current_sub == "graey":
            self.camera = "/auv/camera/videoUSBRaw1"
        elif self.current_sub == "onyx":
            self.camera = "/auv/camera/videoOAKdRawBottom"
        print("[INFO] Template CV init")

    def equalizeHist(self, frame):
        """
        Equalize the histogram of a frame. 

        Args:
            frame (numpy.ndarray): Frame (This is the raw data from the camera feed, and is in BGR format).

        Returns:
            numpy.ndarray: Equalized frame.
        """
        # Split into B, G, and R channels, equalize the histograms for each, then remerge to create a single image.
        frame_b, frame_g, frame_r = cv2.split(frame)
        frame_g = cv2.equalizeHist(frame_g)
        frame_b = cv2.equalizeHist(frame_b)
        frame_r = cv2.equalizeHist(frame_r)
        frame = cv2.merge((frame_b, frame_g, frame_r))
        return frame

    def alignMidpoint(self, midpoint, tolerance):
        """
        Align the sub with the midpoint of the path. Technically the goal is to get the midpoint of the path to x-coordinate 320 in the camera feed, which would 
        be directly in the center of the frame. 

        Args:
            midpoint (int/float): Pixel coordinate of the midpoint of the path in the frame.
            tolerance (int/float): Error tolerance.

        Returns:
            int: Lateral motion command to align the sub with the path.
        """
        if midpoint < 320 - tolerance:
            lateral = -1
        elif midpoint > 320 + tolerance:
            lateral = 1
        else:
            lateral=0
        
        return lateral
    def run(self, frame, target, oakdData=None):
        """
        Run the Follow the Path CV.

        Args:
            frame: The frame from the camera.
            target: This could be any type of information, for example the thing to look for. In this case it is unnecessary.
            oakd_data: The list of detections outputted from the ML model. Since there is no ML model used, there is no OAK-D data necessary/avaliable. Defaults to None.

        Returns:
            dictionary, numpy.ndarray: {lateral motion command, forward motion command, yaw command, flag/state of mission}, visualized frame from the camera
        """
        end = False

        # into_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # frame = self.equalizeHist(frame)
        
        # Filter out the image until all that is left is the orange objects.

        # Convert to grayscale and HSV color spaces.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        into_hsv =(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))

        # Define limits for the color range.
        L_limit=np.array([3, 20, 20]) 
        U_limit=np.array([80, 255, 255])

        # L_limit=np.array([5, 25, 50])
        # U_limit=np.array([30, 255, 255])
        # racquet club testing:
        # L_limit = np.array([8, 100, 100])
        # U_limit = np.array([50, 255, 255])
        # for more range
        # L_limit = np.array([3, 25, 20])
        # U_limit = np.array([50, 255, 255])

        # Create a kernel matrix for morphological (image processing/segmentation/enhancement) operations.
        kernel = np.ones((5, 5), np.uint8)

        # Isolate orange pixels by creating a binary mask: all orange objects will be turned white, all other pixels will be turned black.
        orange=cv2.inRange(into_hsv,L_limit,U_limit)
        
        # Find the edges. Then further refine the edges by using binary thresholding and OTSU's thresholding method. Thresh3 stores the thresholded image.
        edges = cv2.Canny(gray, threshold1=100, threshold2=200)
        ret, thresh3 = cv2.threshold(edges, 230, 255, cv2.THRESH_BINARY) # Turns all pixels with intensity higher than 230 white, and the rest black.
        ret2, thresh3 = cv2.threshold(thresh3, 1, 255, cv2.THRESH_OTSU) # Automatically computes the optimal threshold value based on the histogram. Above the threshold white, below, black.

        # cv2.imshow("edges", thresh3)

        # Reducing noise of the mask.
        orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel)
        time.sleep(0.2)

        # Gaussian blur to smooth out the image and reduce noise. Then threshold the image again (this is not redundant however, as we are thresholding different things as before).
        orange = cv2.GaussianBlur(orange, (11, 11), 0)
        ret, thresh = cv2.threshold(orange, 230, 255, cv2.THRESH_BINARY)

        # Do the same thing as above but use a regular blur, not a Gaussian blur, and threshold using OTSU's method.
        blur = cv2.blur(thresh, (10, 10))
        ret2, thresh2 = cv2.threshold(blur, 1, 255, cv2.THRESH_OTSU)

        # Find contours of the image.
        contours, heirarchy = cv2.findContours(thresh3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, 255, 3)

        yaw = lateral = forward = 0
        hline = cv2.line(frame, (640, 0), (0, 0), (0, 255, 255), 8)

        # If the path exists (if there are contours, meaning there is an outlined image of the path).
        if contours:
            # Find the max contour and draw it.
            c = max(contours, key=cv2.contourArea)
            cv2.drawContours(frame, c, -1, 255, 3)

            # Find the center of the contour, or more accurately find the centroid (center of mass) using moments.
            # Moments are numerical descriptors of the shape, size, and distribution of pixels in an image. In this case we take the all of the moments of the image,
            # and look for the total intensity of the area (m00), the center of mass relative to the x-axis (m10) and to the y-axis (m01).
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(frame, "center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            rows, cols = frame.shape[:2]

            # Fit a line to the path.
            [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)

            lefty = int((-x * vy / vx) + y) # Point on the line where the line intersects the left edge of the image.
            righty = int(((cols - x) * vy / vx) + y) # Point on the line where the line intersects the right edge of the image.

            cv2.line(frame, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)
            cv2.line(frame, (righty, cols - 1), (lefty, 0), (0, 255, 0), 2)

            # Calculate the slope of the line (that is oriented in the direction of the path).
            slope = (lefty - righty) / (0 - (cols - 1))

            # cv2.putText(frame, str(slope), (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            dist = abs(cx - 320) / 320 # Normalize the distance between the centroid of the contour's x-coordinate and the size center x-coordinate of the frame.

            # Strafe (move laterally while keeping a forward facing direction), then yaw after there is no need to move laterally anymore.
            if(self.state == "strafe"):
                lateral = self.alignMidpoint(cx, 20)
                if(lateral == 0):
                    self.state = "yaw"
                else:
                    lateral = np.clip(lateral * dist * 4, -1, 1)
            elif(self.state == "yaw"):
                if -10 < slope < 0:
                    print("yaw clockwise")
                    yaw = 0.65
                elif 0 < slope < 10:
                    print("yaw counter-clockwise")
                    yaw = -0.65
                # If we are aligned, then move forward on the path.
                else:
                    forward=2
                
        elif self.aligned == True:
            end = True
        else:
            forward = 1

        return {"lateral": lateral, "forward": forward, "yaw": yaw, "end": end}, frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly.
    # It is here for testing purposes.
    # You can run this file independently using: "python -m auv.cv.path_cv".

    # Create a CV object with arguments
    cv = CV()

    cap = cv2.VideoCapture("../../testing_data/path_footage.MKV")

    while True:
        ret, img = cap.read()
        if not ret:
            break

        # Set the size of the frame.
        img = cv2.resize(img, (640, 480))

        # Run the CV script.
        result, img_viz = cv.run(img, None, None)

        # Show the result.
        cv2.imshow("frame", img_viz)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
