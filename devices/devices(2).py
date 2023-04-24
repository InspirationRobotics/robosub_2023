import rospy
import cams
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

def get_pub(n, p):
    return p[n][2]

publishers = {
    "cam_down": ["/auv/devices/cam_down", Image, None],
    "cam_front": ["/auv/devices/cam_front", Image, None]
}

cams = 1

def main():
    rospy.init_node("devices_master", anonymous=True)

    if cams:
        bridge = CvBridge()
        if init_cam("front", 0):
            while not rospy.is_shutdown():
                f = get_frame("front")
                show(f)
                msg = bridge.cv2_to_imgmsg(f, encoding="passthrough")
                get_pub("cam_front", publishers).publish(msg)
