import os
import time

from auv.mission import gate_mission
from auv.utils import arm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# load sub config
config = deviceHelper.variables

arm.arm()

# create the mission object
gateMission = gate_mission.GateMission("earth")

# run the mission
gateMission.run()

# terminate the mission
gateMission.cleanup()

# end
print("[INFO] Mission ended")
