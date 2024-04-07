"""
Runs the torpedo mission
"""

import os
import time

from auv.mission import torpedo_mission
from auv.utils import arm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# Load the configuration of the sub's devices
config = deviceHelper.variables

# Arms the sub (sets mode to autonomous)
arm.arm()
time.sleep(5)

# Create the mission object
torpedoMission = torpedo_mission.TorpedoMission(**config)

# Run the mission
torpedoMission.run()

# =Terminate the mission
torpedoMission.cleanup()

# End
print("[INFO] Mission ended")
