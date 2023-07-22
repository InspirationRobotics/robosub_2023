"""
CV Handler
Author: Maxime Ellerbach
"""
import lsb_release

if lsb_release.get_distro_information()["RELEASE"] == "18.04":
    import ctypes

    libgcc_s = ctypes.CDLL("libgcc_s.so.1")

import json
import os
import sys
import threading
import time
import traceback

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CVHandler:
    def __init__(self, **config):
        """Init of CV Handler"""
        self.config = config

        # stores the running cv scripts keyed by file name
        self.active_cv_scripts = {}
        self.subs = {}

    def start_cv(self, file_name, callback, dummy_camera=None):
        """Start a CV script"""
        if file_name in self.active_cv_scripts:
            print("[ERROR] [cvHandler] Cannot start a script that is already running")
            return

        try:
            # module name is as follows: auv.cv.file_name
            module = __import__(f"auv.cv.{file_name}", fromlist=["CV"])
        except Exception as e:
            print("[ERROR] [cvHandler] Error while importing CV module from file name")
            print(f"[ERROR] {e}")
            return

        # Import the CV class from the module
        cv_class = getattr(module, "CV", None)
        if cv_class is None:
            print("[ERROR] [cvHandler] No CV class found in file, check the file name and file content")
            return

        if dummy_camera:  # Init dummy cv script handler
            self.active_cv_scripts[file_name] = _DummyScriptHandler(file_name, cv_class(**self.config), dummy_camera)
        else:  # Init individual cv script handler
            self.active_cv_scripts[file_name] = _ScriptHandler(file_name, cv_class(**self.config))
        self.subs[file_name] = rospy.Subscriber(f"auv/cv_handler/{file_name}", String, callback)

    def stop_cv(self, file_name):
        """Stop a CV script"""
        if file_name not in self.active_cv_scripts:
            print("[ERROR] [cvHandler] Cannot stop a script that is not running")
            return

        self.active_cv_scripts[file_name].stop()
        del self.active_cv_scripts[file_name]

        self.subs[file_name].unregister()
        del self.subs[file_name]

    def switch_oakd_model(self, file_name, model_name):
        if file_name not in self.active_cv_scripts:
            print("[ERROR] [cvHandler] Cannot change model of a script that is not running")
            return

        if not self.active_cv_scripts[file_name].is_oakd:
            print("[ERROR] [cvHandler] Cannot change model of a script that is not running on an OAK-D camera")
            return

        if not isinstance(model_name, str):
            print("[ERROR] [cvHandler] Model name must be a string")
            return

        self.active_cv_scripts[file_name].pub_oakd_model.publish(model_name)
        print(f"[INFO] model published {model_name}")

    def set_target(self, file_name, target):
        if file_name not in self.active_cv_scripts:
            print("[ERROR] [cvHandler] Cannot change target of a script that is not running")
            return

        self.active_cv_scripts[file_name].target = target


class _ScriptHandler:
    def __init__(self, file_name, cv_object):
        self.cv_object = cv_object
        self.file_name = file_name

        # Get the camera topic, if not specified, use the default front camera
        self.camera_topic = getattr(self.cv_object, "camera", None)
        if self.camera_topic is None:
            print("[WARN] No camera topic specified, using default front camera")
            self.camera_topic = "/auv/camera/videoUSBRaw0"

        # Create a cv bridge
        self.br = CvBridge()

        # Create the ROS node, the subscribers and the publishers
        self.sub_cv = rospy.Subscriber(self.camera_topic, Image, self.callback_cam)
        self.pub_viz = rospy.Publisher(self.camera_topic.replace("Raw", "Output"), Image, queue_size=10)
        self.pub_out = rospy.Publisher(f"auv/cv_handler/{file_name}", String, queue_size=10)
        self.pub_cam_select = rospy.Publisher("/auv/camsVersatile/cameraSelect", String, queue_size=10)

        # init the oakd stuff
        if "OAKd" in self.camera_topic:
            self.is_oakd = True
            pub_oakd_model_topic = self.camera_topic.replace("Raw", "Model")
            pub_oakd_data_topic = self.camera_topic.replace("Raw", "Data")
            self.pub_oakd_model = rospy.Publisher(pub_oakd_model_topic, String, queue_size=10)
            self.sub_oakd_data = rospy.Subscriber(pub_oakd_data_topic, String, self.callback_oakd_data)
            time.sleep(1)  # wait for the ros publisher / subscriber to be ready
        else:
            self.is_oakd = False
            self.pub_oakd_model = None
            self.sub_oakd_data = None

        self.initCameraStream()  # sends json to camsVersatile for which camera to start and with model or not

        self.target = "main"
        self.oakd_data = None
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
            print("[ERROR] [cvHandler] Error while converting image to cv2")
            print(f"[ERROR] {e}")

    def callback_oakd_data(self, msg):
        """Callback for the oakd data subscriber"""
        try:
            dataList = []
            data = json.loads(msg.data)
            for detections in data.values():
                dataList.append(Detection(detections))
            self.oakd_data = dataList
        except Exception as e:
            print("[ERROR] [cvHandler] Error while converting oakd data to json")
            print(f"[ERROR] {e}")
            return

    def initCameraStream(self):
        topic = self.camera_topic
        toSend = {}
        if not self.is_oakd:
            self.camID = int(topic[-1])  # by default the low-light cameras are ID'd as 0 for forward and 1 for bottom so this matches instantly
        else:
            if (
                "Forward" in topic
            ):  # the ID>=10 helps camsVers determine it is an oak-d; OakDs are ID'd after lowlight (i.e lowlight, lowlight, oakd) so this becomes a simple operation of id/10 + amount of low-lights-1
                self.camID = 10  # so for Onyx this would be ID 1 since 10/10 = 1 and low-lights (there is 1) - 1=0. For Graey with two low-lights, it is 10/10 = 1 then + 2-1=1 so ID 2
            elif "Bottom" in topic:
                self.camID = 20  # the multiples of 10 allow me to do the same simple equation and still get the right matching ID
            elif "Poe" in topic:
                self.camID = 30
            model = getattr(self.cv_object, "model", None)
            if model is None:
                print("[INFO] No oakD model specified, defaulting to raw")
                model = "raw"
            toSend["model"] = model
        toSend["camera_ID"] = self.camID
        toSend["mode"] = "start"
        data = json.dumps(toSend)
        self.pub_cam_select.publish(data)

    def killCameraStream(self, killAll=False):
        if killAll:
            data = json.dumps({["kill"]: True})
        else:
            toSend = {["camera_id"]: self.camID, ["mode"]: "stop"}
            data = json.dumps(toSend)
        self.pub_cam_select.publish(data)

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
                ret = self.cv_object.run(frame, self.target, self.oakd_data)
            except Exception as e:
                print(f"[ERROR] [cvHandler] Error while running CV {self.file_name} {e}")
                print(e)
                continue

            if isinstance(ret, tuple) and len(ret) == 2:
                result, viz_img = ret
            elif isinstance(ret, dict):
                result = ret
                viz_img = None
            else:
                print("[ERROR] [cvHandler] CV returned invalid type")
                continue

            # Publish the result
            self.pub_out.publish(json.dumps(result))

            if viz_img is not None:
                self.pub_viz.publish(self.br.cv2_to_imgmsg(viz_img))

    def stop(self):
        self.running = False
        self.thread.join()
        # self.killCameraStream() # commented out temporarily so we can continue watching stream post mission
        self.sub_cv.unregister()
        self.pub_viz.unregister()
        self.pub_out.unregister()
        self.pub_cam_select.unregister()
        self.closed = True


class _DummyScriptHandler:
    def __init__(self, file_name, cv, dummy):
        self.file_name = file_name
        self.cv = cv

        # Get the camera topic, if not specified, use the default front camera
        self.camera_topic = getattr(self.cv_object, "camera", None)
        if self.camera_topic is None:
            print("[WARN] No camera topic specified, using default front camera")
            self.camera_topic = "/auv/camera/videoUSBRaw0"

        self.is_oakd = False
        self.target = "main"
        self.oakd_data = None
        self.running = False
        self.closed = False

        # this is our dummy camera
        self.dummy_video = dummy
        self.cap = cv2.VideoCapture(self.dummy_video)
        if not self.cap.isOpened():
            print("[ERROR] [cvHandler] Error while opening dummy video")
            return

        # run the video in a thread, loop when it ends
        self.thread = threading.Thread(target=self.run)

        # should we post the viz to the viz topic?
        self.pub_viz = rospy.Publisher(self.camera_topic.replace("Raw", "Output"), Image, queue_size=10)
        self.pub_out = rospy.Publisher(f"auv/cv_handler/{file_name}", String, queue_size=10)

    def run(self):
        self.running = True

        while self.running:
            ret, frame = self.cap.read()

            # loop if video ends
            if not ret:
                self.cap = cv2.VideoCapture(self.dummy_video)
                continue

            # Run the CV
            try:
                ret = self.cv_object.run(frame, self.target, self.oakd_data)
            except Exception as e:
                traceback.print_exception()
                print(f"[ERROR] [cvHandler] Error while running CV {self.file_name}")
                continue

            if isinstance(ret, tuple) and len(ret) == 2:
                result, viz_img = ret
            elif isinstance(ret, dict):
                result = ret
                viz_img = None
            else:
                print("[ERROR] [cvHandler] CV returned invalid type")
                continue

            # Publish the result
            self.pub_out.publish(json.dumps(result))

            if viz_img is not None:
                self.pub_viz.publish(self.br.cv2_to_imgmsg(viz_img))

    def stop(self):
        self.running = False
        self.thread.join()

        self.pub_viz.unregister()
        self.pub_out.unregister()
        self.closed = True


class Detection:
    def __init__(self, data):
        self.label = data[0]
        self.confidence = data[1]
        self.xmin = data[2]
        self.xmax = data[3]
        self.ymin = data[4]
        self.ymax = data[5]


if __name__ == "__main__":
    # some small testings for CVHandler, requires camVersatile to run before

    def dummy_callback(msg):
        print(f"[INFO] received: {msg.data}")

    cv = CVHandler()
    cv.start_cv("template_cv")
