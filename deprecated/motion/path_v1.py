import rospy
from insp_msgs.msg import PointArray
from std_msgs.msg import Int32MultiArray


def get_pub(n, p):
    return p[n][2]


def pwm_cb(p):
    # 	if sd['mode'] == "raw":
    get_pub("thrusters", publishers).publish(p)


publishers = {"raw": ["/auv/motion/raw", Int32MultiArray, None]}
subscribers = {"path": ["/auv/cv/path", PointArray, mode_cb, None]}
sd = {"path": None}


def init_ros_io(p, s):
    for i in s.values():
        i[3] = rospy.Subscriber(i[0], i[1], i[2], queue_size=10)

    for i in p.values():
        i[2] = rospy.Publisher(i[0], i[1], queue_size=10)


def SDoF(th, fw, lat, yaw, pitch, roll):
    get_pub("raw", publishers).publish(
        [
            1500,
            1500,
            th,
            yaw,
            fw,
            pitch,
            roll,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
        ]
    )


def main():
    rospy.init_node("path_v1", anonymous=True)

    while not rospy.is_shutdown():
        if sd["path"] == None:
            SDof(1500, 1500, 1500, 1500, 1500, 1500)
        else:
            SDoF(1500, 1500, 1500, 1700, 1500, 1500)

    print("finished executing path mission")
    rospy.spin()
