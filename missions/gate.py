"""
Runs the Gate mission
"""

import os
import time

from auv.mission import gate_mission
from auv.utils import arm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# Load the configuration of the sub's devices
config = deviceHelper.variables

# Arm the sub (set mode to autnomous)
arm.arm()

# Create the mission object
gateMission = gate_mission.GateMission("earth")

# Run the mission
gateMission.run()

# Terminate the mission
gateMission.cleanup()

# End
print("[INFO] Mission ended")
