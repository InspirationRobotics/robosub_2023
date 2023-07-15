"""
CV Handler
"""

import json
import logging
import os
import threading
import time

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class _ScriptHandler:
    def __init__(self, file_name, cv_object, camera_topic):
        self.cv_object = cv_object
        self.camera_topic = camera_topic
        self.file_name = file_name

        # Create a cv bridge
        self.br = CvBridge()

        # Create the ROS node, the subscribers and the publishers
        self.sub_cv = rospy.Subscriber(self.camera_topic, Image, self.callback_cam)
        self.pub_viz = rospy.Publisher(self.camera_topic.replace("Raw", "Output"), Image, queue_size=10)
        self.pub_out = rospy.Publisher("auv/cv_handler/{}".format(file_name), String, queue_size=10)

        if "OAKd" in self.camera_topic:
            self.is_oakd = True
            self.pub_oakd_model = rospy.Publisher(self.camera_topic.replace("Raw", "Model"), String)
            self.sub_oakd_data = rospy.Subscriber(self.camera_topic.replace("Raw", "Data"), String, self.callback_oakd_data)
        else:
            self.is_oakd = False
            self.pub_oakd_model = None
            self.sub_oakd_data = None

        self.next_frame = None
        self.running = False
        self.closed = False
        self.last_received = time.time()
        self.last_processed = self.last_received

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def callback_cam(self, msg):
        """Callback for the cam subscriber"""
        try:
            self.next_frame = self.br.imgmsg_to_cv2(msg)
            self.last_received = time.time()
        except Exception as e:
            logger.error("Error while converting image to cv2")
            logger.error(e)

    def callback_oakd_data(self, msg):
        """Callback for the oakd data subscriber"""
        try:
            data = json.loads(msg.data)
            # TODO: do something with the data
            logger.warning("OAK-D data received but not implemented yet")
        except Exception as e:
            logger.error("Error while converting oakd data to json")
            logger.error(e)
            return

    def run(self):
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
            try:
                ret = self.cv_object.run(frame)
            except Exception as e:
                logger.error("Error while running CV {} {}".format(self.file_name, e))
                continue

            if isinstance(ret, tuple) and len(ret) == 2:
                result, viz_img = ret
            elif isinstance(ret, dict):
                result = ret
                viz_img = None
            else:
                logger.error("CV returned invalid type")
                continue

            # if 

            # Publish the result
            self.pub_out.publish(json.dumps(result))

            if viz_img is not None:
                self.pub_viz.publish(self.br.cv2_to_imgmsg(viz_img))

    def stop(self):
        self.running = False
        self.thread.join()

        self.sub_cv.unregister()
        self.pub_viz.unregister()
        self.pub_out.unregister()
        self.closed = True

    def __del__(self):
        if not self.closed:
            self.stop()


class CVHandler:
    def __init__(self, **config):
        """Init of CV Handler"""
        self.config = config

        # stores the running cv scripts keyed by file name
        self.active_cv_scripts = {}
        self.subs = {}

    def start_cv(self, file_name, callback):
        """Start a CV script"""
        if file_name in self.active_cv_scripts:
            logger.error("Cannot start a script that is already running")
            return

        try:
            # module name is as follows: auv.cv.file_name
            module = __import__("auv.cv.{}".format(file_name), fromlist=["CV"])
        except Exception as e:
            logger.error("Error while importing CV module from file name")
            logger.error(e)
            return

        # Import the CV class from the module
        cv_class = getattr(module, "CV", None)
        if cv_class is None:
            logger.error("No CV class found in file, check the file name and file content")
            return

        # Get the camera topic, if not specified, use the default front camera
        camera_topic = getattr(cv_class, "camera", None)
        if camera_topic is None:
            logger.warning("No camera topic specified, using default front camera")
            camera_topic = "/auv/camera/videoUSBRaw0"

        # Init individual cv script handler
        self.active_cv_scripts[file_name] = _ScriptHandler(file_name, cv_class(**self.config), camera_topic)
        self.subs[file_name] = rospy.Subscriber("auv/cv_handler/{}".format(file_name), String, callback)

    def stop_cv(self, file_name):
        """Stop a CV script"""
        if file_name not in self.active_cv_scripts:
            logger.error("Cannot stop a script that is not running")
            return

        self.active_cv_scripts[file_name].stop()
        del self.active_cv_scripts[file_name]

        self.subs[file_name].unregister()
        del self.subs[file_name]

    def change_oakd_model(self, file_name, model_name):
        if file_name not in self.active_cv_scripts:
            logger.error("Cannot change model of a script that is not running")
            return

        if not self.active_cv_scripts[file_name].is_oakd:
            logger.error("Cannot change model of a script that is not running on an OAK-D camera")
            return

        if not isinstance(model_name, str):
            logger.error("Model name must be a string")
            return

        self.active_cv_scripts[file_name].pub_oakd_model.publish(model_name)


if __name__ == "__main__":
    # some small testings for CVHandler, requires camVersatile to run before

    logging.basicConfig()

    def dummy_callback(msg):
        logger.info("received: {}".format(msg.data))

    cv = CVHandler()
    cv.start_cv("template_cv")
