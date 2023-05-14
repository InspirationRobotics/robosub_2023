# a very simple mission that moves the sub in a square (read the corresponding tutorial)

import rospy
from std_msgs.msg import Int32MultiArray

import time

# name, topic, data type, publisher
publishers = [
				["raw", "/auv/motion/raw", Int32MultiArray, NULL]
]

# ignore while there are no subscribers
# global_data = { 'x': [] }
# def x_cb(a):
#	global_data['x'] = a.data
subscribers = [
				# ["/a/b", String, x_cb, NULL]
]

def init_ros(p, s, node_name):
	for i in p:
		i[3] = rospy.Publisher(i[1], i[2], queue_size=10)
	for i in s:
		i[3] = rospy.Subscriber(i[0], i[1], i[2], queue_size=10)
		rospy.init_node(node_name, anonymous=True)

def pub(name):
	for i in publishers:
		if i[0] == name:
			return i[3]

def pwm_data(c, pwm):
	data = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
	data[c] = pwm

	return data

def main():
	init_ros(publishers, subscribers, "baseline_mission")

	pub('raw').publish(gen_channels(3, 1700)) 
	time.sleep(3)
	pub('raw').publish(gen_channels(5, 1700))
	time.sleep(3)
	pub('raw').publish(gen_channels(3, 1300))
	time.sleep(3)
	pub('raw').publish(gen_channels(5, 1300))
	pub('raw').publish(gen_channels(5, 1500))

	rospy.spin()
