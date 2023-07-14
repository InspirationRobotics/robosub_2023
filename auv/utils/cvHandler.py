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


class ScriptHandler:
    def __init__(self, file_name, cv_object, camera_topic):
        self.cv_object = cv_object
        self.camera_topic = camera_topic
        self.file_name = file_name

        # Create the ROS node, the subscribers and the publishers
        self.br = CvBridge()
        self.sub_cv = rospy.Subscriber(self.camera_topic, Image, self.callback_cam)
        self.pub_viz = rospy.Publisher(self.camera_topic.replace("Raw", "Output"), Image, queue_size=10)
        self.pub_out = rospy.Publisher("auv/cv_handler/{}".format(file_name), String, queue_size=10)

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

    def run(self, frame):
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

        # Create the ROS node and the cmd subscriber
        rospy.init_node("cv_handler", anonymous=True)
        self.sub_cmd = rospy.Subscriber("auv/cv_handler/cmd", String, self.callback_cmd)
        
        # stores the running cv scripts keyed by file name
        self.active_cv_scripts = {}

    def callback_cmd(self, msg):
        """Callback for the cmd subscriber"""
        try:
            cmd = json.loads(msg.data)
            file_name = cmd.get("file_name", None)
            stop_file = cmd.get("stop", None)  # we just received a stop command

            if stop and stop_file in self.active_cv_scripts:
                self.active_cv_scripts[stop_file].stop()
                del self.active_cv_scripts[stop_file]
                return

            elif stop:
                logger.error("Cannot stop a script that is not running")

            if file_name is None:
                logger.error("No file name given")
                return
            
            # Check if the file exists
            if not os.path.isfile(file_name):
                logger.error("File does not exist")
                return

            # Import the CV class from the file_name
            # module name is as follows: auv.cv.file_name
            module = __import__(file_name, fromlist=[""])
            cv_class = getattr(module, "CV")

            # Get the camera topic, if not specified, use the default front camera
            camera_topic = getattr(cv_class, "camera", None)
            if camera_topic is None:
                logger.warning("No camera topic specified, using default front camera")
                camera_topic = "/auv/camera/videoUSBRaw0"

            # Init individual cv script handler
            self.running[file_name] = ScriptHandler(cv_class(**self.config), camera_topic)

        except Exception as e:
            logger.error("Error while converting cmd to json")
            logger.error(e)

if __name__ == "__main__":
    # spinup the cv handler with the surfacing cv

    from auv.cv import surfacing_cv, template_cv

    cv = CVHandler()
