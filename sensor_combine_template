# ROS 1 Template for combining sensor data
# write what this node is for ex: combining GPS signal/location
#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

# name the node with the sensor name, make sure to differentiate the nodes with something like numbering 
def sensor1_callback(data):
    rospy.loginfo(data.data)

def sensor2_callback(data):
    rospy.loginfo(data.data)
# add as many functions as you need for this sensor, one function per topic you need to listen to/get data from

# create a node to combine the data
def sensor_combine():
    pub = rospy.Publisher('/wamv/sensors/sensor/data', String, queue_size=10)
    rospy.init_node('sensor_combine', anonymous=True)
    rospy.Subscriber("/wamv/sensors/sensor/data1", String, sensor1_callback)
    rospy.Subscriber("/wamv/sensors/sensor/data2", String, sensor2_callback)
    # add the same amount of subscriber nodes as the nodes you made in the top portion of this file 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time() # can edit the string name and output to reflect the data you are getting from the sensor and perform different operations like adding the two pieces of sensor data together when printing the string
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_combine()
    except rospy.ROSInterruptException:
        pass
