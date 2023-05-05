import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

def get_pub(n, p):
        return p[n][2]

publishers = {
        "thrusters": ["/auv/devices/thrusters", Int32MultiArray, None]
}

sd = {
	"mode": []
}

def pwm_cb(p): 
#	if sd['mode'] == "raw":
        rospy.loginfo(p.data)
        get_pub("thrusters", publishers).publish(p)

def mode_cb(p):
        pass

subscribers = {
	"mode": ["/auv/motion/mode", String, mode_cb, None],
	"raw": ["/auv/motion/raw", Int32MultiArray, pwm_cb, None], # basically just passthrough
}

def init_ros_io(p, s):
        for i in s.values():
                i[3] = rospy.Subscriber(i[0], i[1], i[2], queue_size=10)
        for i in p.values():
                i[2] = rospy.Publisher(i[0], i[1], queue_size=10)
 

def main():
        print("entered main")
        rospy.init_node("motion_handler", anonymous=True)
        init_ros_io(publishers, subscribers)
        print("initialized motion handler")
        rospy.spin()

if __name__ == "__main__":
   while not rospy.is_shutdown():
      main()
