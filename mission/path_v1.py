import rospy
from std_msgs.msg import Int32MultiArray
from insp_msgs.msg import PointArray

def get_pub(n, p):
        return p[n][2]

publishers = {
        "raw": ["/auv/motion/raw", Int32MultiArray, None]
}

sd = {
	"path": None,
}

def pwm_cb(p): 
#	if sd['mode'] == "raw":
        get_pub("thrusters", publishers).publish(p)

subscribers = {
	"path": ["/auv/cv/path", PointArray, mode_cb, NULL],
}

def init_ros_io(p, s):
	for i in s:
                i[4] = rospy.Subscriber(i[1], i[2], i[3], queue_size=10)

        for i in p:
		i[3] = rospy.Publisher(i[1], i[2], queue_size=10)

def SDoF(th, fw, lat, yaw, pitch, roll):
    get_pub("raw", publishers).publish([1500, 1500, th, yaw, fw, pitch, roll, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
                
def main():
    rospy.init_node("path_v1", anonymous=True)

    while not rospy.is_shutdown():
        if sd["path"] == None:
            SDof(1500, 1500, 1500, 1500, 1500, 1500)
        else:
            SDoF(1500, 1500, 1500, 1700, 1500, 1500)
    
    print("finished executing path mission")
    rospy.spin()
