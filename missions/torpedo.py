import os
import time

from auv.mission import torpedo_mission
from auv.utils import arm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# load sub config
config = deviceHelper.variables

arm.arm()
time.sleep(5)

# create the mission object
torpedoMission = torpedo_mission.TorpedoMission(**config)

# run the mission
torpedoMission.run()

# terminate the mission
torpedoMission.cleanup()

# end
print("[INFO] Mission ended")
