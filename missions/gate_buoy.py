from auv.mission import gate_mission, buoy_mission
from auv.utils import arm, disarm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# load sub config
config = deviceHelper.variables
arm.arm()

target = "abydos"

gateMission = gate_mission.GateMission(target)
gateMission.run()
gateMission.cleanup()

# TODO: fix A2 stuff within buoy mission
buoyMission = buoy_mission.BuoyMission("A2")
buoyMission.run()
buoyMission.cleanup()