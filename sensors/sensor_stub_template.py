# ROS 1 Template for sensor stubs
# write what this node is for ex: reading data from individual GPS 
#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

# name your node
def sensor1_function1():
    # in the single quotes write where your node is publishing to
    pub = rospy.Publisher('/wamv/sensors/sensor1/data', String, queue_size=10)
    rospy.init_node('sensor1_function1', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor1_function1()
    except rospy.ROSInterruptException:
        pass
