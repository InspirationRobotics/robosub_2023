import time

from auv.mission import surfacing_mission
from auv.utils import arm, disarm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# load sub config
config = deviceHelper.variables
arm.arm()

# create the mission object
surfacingMission = surfacing_mission.SurfacingMission(**config)

# run the mission
surfacingMission.run()

# terminate the mission
surfacingMission.cleanup()

# end
print("[INFO] Mission ended")
disarm.disarm()