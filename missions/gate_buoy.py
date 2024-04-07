"""
Runs the Gate mission then the Buoy mission
"""

from auv.mission import gate_mission, buoy_mission
from auv.utils import arm, disarm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# Load the configuration of the sub's devices
config = deviceHelper.variables
arm.arm()

# Set the target side of the gate to be Abydos
target = "abydos"

# Run the Gate mission
gateMission = gate_mission.GateMission(target)
gateMission.run()
gateMission.cleanup()

# Run the Buoy mission
# TODO: fix A2 stuff within buoy mission
buoyMission = buoy_mission.BuoyMission("A2")
buoyMission.run()
buoyMission.cleanup()