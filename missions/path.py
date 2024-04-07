"""
Run the Follow the Path mission
"""

import os
import time

from auv.mission import path_mission
from auv.utils import arm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# Load the configuration of the sub's devices
config = deviceHelper.variables

# arm.arm()

# Create the mission object
pathMission = path_mission.PathMission(**config)

# Run the mission
pathMission.run()

# Terminate the mission
pathMission.cleanup()

# End
print("[INFO] Mission ended")
