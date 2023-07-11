"""
CV class for surfacing
"""

import logging

import cv2
import numpy as np

try:
    import rospy
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray
except ImportError:
    # if you are not on the auv, you can still run this file
    # but you won't be able to use the ROS stuff
    pass

logger = logging.getLogger(__name__)


def get_octogon_center(self):
    """
    Returns the center of the octogon in the frame
    using a contours detection approach
    """

    # convert to grayscale
    gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

    # apply Canny edge detection
    edges = cv2.Canny(image=gray, threshold1=0, threshold2=200)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)

    # combine with memory edges
    self.memory_edges = cv2.addWeighted(self.memory_edges, 0.85, edges, 0.15, 0)
    _, edges = cv2.threshold(self.memory_edges, 60, 255, cv2.THRESH_BINARY)

    # cv2.imshow("memory_edges", memory_edges)
    # cv2.imshow("edges", edges)

    # find contours, convex hull, and approx poly
    contours, hierarchy = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS
    )
    contours = [cv2.convexHull(contour) for contour in contours]
    contours = [contour for contour in contours if cv2.contourArea(contour) > 1000]

    if contours is None or len(contours) == 0:
        return (None, None)

    # find the largest contour
    largest_contour = max(contours, key=cv2.contourArea)

    # draw contours
    self.frame = cv2.drawContours(self.frame, contours, -1, (0, 255, 0), 2)
    self.frame = cv2.drawContours(frame, [largest_contour], -1, (0, 255, 255), 2)

    # find the center of the contour
    M = cv2.moments(largest_contour)

    if M["m00"] == 0:
        return (None, None)

    x_center = int(M["m10"] / M["m00"])
    y_center = int(M["m01"] / M["m00"])
    self.frame = cv2.circle(frame, (x_center, y_center), 5, (0, 0, 255), -1)
    return (x_center, y_center)


class SurfacingCV:
    def callback(self, data):
        """
        Callback called everytime a new frame is published on the camera topic
        """
        try:
            # Convert the image to OpenCV format
            self.next_frame = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            logger.error(e)

    def __init__(self, **kwargs):
        """
        Init of surfacing CV
        """

        self.frame = None
        self.next_frame = None
        self.memory_edges = None

        # Create a bridge to convert ROS Image format to OpenCV
        self.bridge = CvBridge()

        # Create a node
        rospy.init_node("surfacing_cv", anonymous=True)

        # Create a subscriber to get the frames from the camera
        self.sub = rospy.Subscriber("/auv/camera/videoUSBRaw1", Image, self.callback)
        # Create a publisher to publish back a vizualization of the CV
        self.pub_cv = rospy.Publisher("/auv/camera/videoUSBOutput1", Image)
        # Create a publisher to publish the result of the CV
        self.pub = rospy.Publisher("/auv/cv/surfacing", Float32MultiArray)

        logger.info("Surfacing CV init")

    def run(self):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        logging.info("Surfacing CV run")

        while not rospy.is_shutdown():
            # Wait for the next frame
            if self.next_frame is None:
                continue

            self.frame = self.next_frame
            self.next_frame = None

            # Do the CV
            x_center, y_center = get_octogon_center(self)

            # Publish center of octogon
            if x_center is not None and y_center is not None:
                self.pub.publish(Float32MultiArray(data=[x_center, y_center]))

            # Publish the frame
            self.pub_cv.publish(self.bridge.cv2_to_imgmsg(self.frame))

        return

    def terminate(self):
        # Stop the node
        rospy.signal_shutdown("End of CV surfacing")
        logger.info("Surfacing CV terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template"

    # Create a CV object with arguments
    cv = TemplateCV(arg1="value1", arg2="value2")

    # run the center detection
    get_octogon_center(cv)
