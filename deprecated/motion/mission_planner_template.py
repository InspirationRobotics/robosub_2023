# Template for the node that will be making decisions on what mission it is/when to switch
# Currently this file is a text file, make sure the actual mission planner file name has the correct extension like .py
# Takes in information about mission location and current sub position. Outputs the next waypoint that the sub needs to move to.

#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String


def current_pos_lat_callback(data):
    rospy.loginfo(data.data)
    # write what you will be doing with the callback function in a comment so we can keep track of what we are using

def current_pos_long_callback(data):
    rospy.loginfo(data.data)
    
def current_pos_heading_callback(data):
    rospy.loginfo(data.data)
    
def perception_gate_callback(data):
    rospy.loginfo(data.data)
    
def perception_buoy_callback(data):
    rospy.loginfo(data.data)
    
def perception_path_callback(data):
    rospy.loginfo(data.data)

def perception_marker_callback(data):
    rospy.loginfo(data.data)
    
def perception_torpedoes_callback(data):
    rospy.loginfo(data.data)
    
def perception_oct_callback(data):
    rospy.loginfo(data.data)

def hydrophone_torpedo_callback(data):
    rospy.loginfo(data.data)
    
def hydrophone_oct_callback(data):
    rospy.loginfo(data.data)

def sonar_callback(data):
    rospy.loginfo(data.data)

def dvl_callback(data):
    rospy.loginfo(data.data)

def state_estimator_gate_callback(data):
    rospy.loginfo(data.data)
    
def state_estimator_buoy_callback(data):
    rospy.loginfo(data.data)
    
def state_estimator_path_callback(data):
    rospy.loginfo(data.data)
    
def state_estimator_marker_callback(data):
    rospy.loginfo(data.data)
    
def state_estimator_torpedo_callback(data):
    rospy.loginfo(data.data)
    
def state_estimator_octsurf_callback(data):
    rospy.loginfo(data.data)
    
def state_estimator_octbin_callback(data):
    rospy.loginfo(data.data)

# make sure you have declared the same number of functions as subscribers

def mission_planner():
    lat_target = rospy.Publisher('/wamv/position/current_latitude', String, queue_size=10)
    lon_target = rospy.Publisher('/wamv/position/current_longitude', String, queue_size=10)
    heading_target=rospy.Publisher('/wamv/position/current_heading', String, queue_size=10)
    rospy.init_node('mission_planner', anonymous=True)
    rospy.Subscriber("/wamv/position/current_latitude", String, current_pos_lat_callback)
    rospy.Subscriber("/wamv/position/current_longitude", String, current_pos_long_callback)
    rospy.Subscriber("/wamv/position/current_heading", String, current_pos_heading_callback)
    rospy.Subscriber("/wamv/sensors/perception/gate", String, perception_gate_callback)
    rospy.Subscriber("/wamv/sensors/perception/buoy", String, perception_buoy_callback)
    rospy.Subscriber("/wamv/sensors/perception/path", String, perception_path_callback)
    rospy.Subscriber("/wamv/sensors/perception/marker_bins", String, perception_marker_callback)
    rospy.Subscriber("/wamv/sensors/perception/torpedoes", String, perception_torpedoes_callback)
    rospy.Subscriber("/wamv/sensors/perception/octagon_bins", String, perception_oct_callback)
    rospy.Subscriber("/wamv/sensors/hydrophone/torpedo_beacon", String, hydrophone_torpedo_callback)
    rospy.Subscriber("/wamv/sensors/hydrophone/octagon_beacon", String, hydrophone_oct_callback)
    rospy.Subscriber("/wamv/sensors/sonar/data", String, sonar_callback)
    rospy.Subscriber("/wamv/sensors/dvl/data", String, dvl_callback)
    rospy.Subscriber("/wamv/mission/gate/confidence", String, state_estimator_gate_callback)
    rospy.Subscriber("/wamv/mission/buoy/confidence", String, state_estimator_buoy_callback)
    rospy.Subscriber("/wamv/mission/path/confidence", String, state_estimator_path_callback)
    rospy.Subscriber("/wamv/mission/marker_drop/confidence", String, state_estimator_marker_callback)
    rospy.Subscriber("/wamv/mission/torpedo/confidence", String, state_estimator_torpedo_callback)
    rospy.Subscriber("/wamv/mission/octagon_surface/confidence", String, state_estimator_octsurf_callback)
    rospy.Subscriber("/wamv/mission/octagon_bins/confidence", String, state_estimator_octbin_callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time() # this can be edited to print relevant messages 
        rospy.loginfo(hello_str)
        lat_target.publish(hello_str)
        lon_target.publish(hello_str)
        heading_target.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        mission_planner()
    except rospy.ROSInterruptException:
        pass
