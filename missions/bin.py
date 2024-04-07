"""
Run the Bin mission
"""

from auv.mission import bin_mission
from auv.utils import arm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# Load the configuration for the sub's devices
config = deviceHelper.variables

# Arm the sub
arm.arm() 

# Create the mission object
binMission = bin_mission.BinMission(**config)

# Run the mission
binMission.run()

# Terminate the mission
binMission.cleanup()

# End
print("[INFO] Mission ended")
