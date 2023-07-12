"""
CV Handler
"""

import json
import logging
import os
import time

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class CVHandler:
    def __init__(self, **config):
        """Init of CV Handler"""

        # Create the cv bridge, the ROS node, the subscriber and the publishers
        self.br = CvBridge()
        self.rospy = rospy.init_node("cv_handler", anonymous=True)
        self.sub_cmd = rospy.Subscriber("auv/cv_handler/cmd", String, self.callback_cmd)
        self.pub_out = rospy.Publisher("auv/cv_handler/output", String, queue_size=10)
        self.sub_cv = None
        self.pub_viz = None

        self.next_frame = None
        self.running = False
        self.closed = False
        self.last_received = time.time()
        self.last_processed = self.last_received

    def init_ros(self, cv_class, camera_topic):
        """Init the ROS part of the CV Handler"""
        self.cv_object = cv_class()
        self.camera_topic = camera_topic

        self.sub_cv = rospy.Subscriber(self.camera_topic, Image, self.callback_cam)
        self.pub_viz = rospy.Publisher(self.camera_topic.replace("Raw", "Output"), Image, queue_size=10)

    def callback_cmd(self, msg):
        """Callback for the cmd subscriber"""
        try:
            cmd = json.loads(msg.data)
            module_name = cmd.get("module_name", None)
            camera_topic = cmd.get("camera_topic", None)
            stop = cmd.get("stop", False) # we just received a stop command

            self.running = False
            while not self.closed:
                time.sleep(0.1)
            if stop:
                return

            if module_name is None:
                logger.error("No file name given")
                return

            if camera_topic is None:
                logger.error("No camera topic given")
                return

            # Check if the file exists
            if not os.path.isfile(module_name):
                logger.error("File does not exist")
                return


            # Import the CV class from the module_name
            # module name is as follows: auv.cv.module_name
            module = __import__(module_name, fromlist=[""])
            cv_class = getattr(module, "CV")

            # Init the ROS part
            self.init_ros(cv_class, camera_topic)

            # Run the CV
            self.run()

        except Exception as e:
            logger.error("Error while converting cmd to json")
            logger.error(e)

    def callback_cam(self, msg):
        """Callback for the cam subscriber"""
        try:
            self.next_frame = self.br.imgmsg_to_cv2(msg)
            self.last_received = time.time()
        except Exception as e:
            logger.error("Error while converting image to cv2")
            logger.error(e)

    def run(self):
        """Run the CV Handler"""
        self.closed = False
        self.running = True

        while self.running:
            # Check if the a new frame is received
            if self.next_frame is None:
                continue
            if self.last_received == self.last_processed:
                continue
            self.last_processed = self.last_received

            # Run the CV
            frame = self.next_frame
            ret = self.cv_object.run(frame)

            if isinstance(ret, tuple) and len(ret) == 2:
                result, viz_img = ret
            elif isinstance(ret, dict):
                result = ret
                viz_img = None
            else:
                logger.error("CV returned invalid type")
                continue

            # Publish the result
            self.pub_out.publish(json.dumps(result))

            if viz_img is not None:
                self.pub_viz.publish(self.br.cv2_to_imgmsg(viz_img))

        self.closed = True


if __name__ == "__main__":
    # spinup the cv handler with the surfacing cv

    from auv.cv import surfacing_cv, template_cv

    cv = CVHandler()
    cv.init_ros(surfacing_cv.CV, "/auv/camera/videoUSBRaw0")
    cv.run()
