import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cam, hyd, baro, imu, dvl
from std_msgs.msg import Int16

from enum import Enum

class Sensor(Enum):
	CAM_FRONT = 0
	CAM_DOWN = 1
	HYD = 2
	BARO = 3
	IMU = 4
	DVL = 5
	COMPASS = 6	
	STATUS = 7

publishers = [
		["/auv/sensors/cam_front", Image, NULL],
		["/auv/sensors/cam_down", Image, NULL],
		["/auv/sensors/hyd", Pose, NULL],
		["/auv/sensors/baro", Float64, NULL],
		["/auv/sensors/imu_acc", Float64, NULL],
		["/auv/sensors/dvl", Point, NULL],
		["/auv/sensors/compass", Float64, NULL],
		["/auv/sensors/status", Int16[], NULL]
]

subscribers = []

def init_ros(p, s, node_name):
	for i in p:
		i[2] = rospy.Publisher(i[0], i[1], queue_size=10)
	
	rospy.init_node(node_name, anonymous=True)

def init_sensor(s):
	if s == Sensor.CAM_FRONT:
		return init_cam_front()
	if s == Sensor.CAM_DOWN:         
		return init_cam_down()
	if s == Sensor.HYD:
		return init_hyd()

def read_sensor(s)
	if s == Sensor.CAM_FRONT:
                return init_cam_front()
        if s == Sensor.CAM_DOWN:
                return init_cam_down()
        if s == Sensor.HYD:
                return init_hyd()

def main(): 
	init_ros("sensors")	
	sensorstatus = [0, 0, 0, 0, 0, 0]


	for i in range(0, 6):
		if init_sensor(s) == 0:
			print(Sensor(i).name + " succeeded initializing")
			sensorstatus[i] = 1
		else:
			print(Sensor(i).name + " failed initializing")

	while not rospy.is_shutdown():
		for i in range(0, 6):
			if sensorstatus[i]:
				publishers[i][2].publish(read_sensor(i))
		
		publishers[Sensor.STATUS][2].publish(sensorstatus)
