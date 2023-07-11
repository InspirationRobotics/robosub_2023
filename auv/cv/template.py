"""
Template file to create a CV class
"""

# import what you need from within the package

import logging
import time

import cv2

try:
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray
    ROS = True
except ImportError:
    # if you are not on the auv, you can still run this file
    # but you won't be able to use the ROS stuff
    ROS = False
    pass

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def some_cv_function(self):
    """
    Some function that does some cv stuff
    """
    logger.info("some_cv_function")
    return True


class TemplateCV:
    def callback(self, data):
        """
        Callback called everytime a new frame is published on the camera topic
        """
        try:
            # Convert the image to OpenCV format
            self.frame = self.bridge.imgmsg_to_cv2(data)
        except Exception as e:
            logger.error(e)

    def __init__(self, **kwargs):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        kwargs is a dict containing the key arguments you give to the CV
        """

        self.frame = None
        self.running = True

        if ROS:
            # Create a bridge to convert ROS Image format to OpenCV
            self.bridge = CvBridge()

            # Create a node
            rospy.init_node("template_cv", anonymous=True)

            # Create a subscriber to get the frames from the camera
            self.sub = rospy.Subscriber("/auv/camera/videoUSBRaw0", Image, self.callback)
            # Create a publisher to publish back a vizualization of the CV
            self.pub_viz = rospy.Publisher("/auv/camera/videoUSBOutput0", Image)
            # Create a publisher to publish the result of the CV
            self.pub = rospy.Publisher("/auv/cv/template", Float32MultiArray)

        logger.info("Template CV init")

    def run(self):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        logging.info("Template CV run")

        while not rospy.is_shutdown():
            # do stuff
            time.sleep(1.0)
            break

        return

    def terminate(self):
        """
        Here should be all the code required to terminate the mission.
        This could be cleanup, saving data, closing files, etc.
        Return True if everything went well, False otherwise (ideally)
        """
        logger.info("Template CV terminate")
        return True


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template"

    logging.basicConfig()

    # Create a CV object with arguments
    cv = TemplateCV(arg1="value1", arg2="value2")

    # here you can for example initialize your camera, etc
    # cap = cv2.VideoCapture(0)
    # ret, frame = cap.read()
    # cv.frame = frame
    # ...

    # run a cv function
    some_cv_function(cv)

