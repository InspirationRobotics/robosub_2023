import rospy
import mavros_msgs.msg
import mavros_msgs.srv
import geographic_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

def get_pub(n, p):
        return p[n][2]

publishers = {
        "thrusters": ["/auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn, None]
}

sd = {
	"mode": []
}

def pwm_cb(p): 
#	if sd['mode'] == "raw":
        rospy.loginfo(p.data)
        print(p)
        get_pub("thrusters", publishers).publish(p)

def mode_cb(p):
        pass

subscribers = {
	"mode": ["/auv/status/mode", std_msgs.msg.String, mode_cb, None],
	"raw": ["/auv/motion/raw", mavros_msgs.msg.OverrideRCIn, pwm_cb, None], # basically just passthrough
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
