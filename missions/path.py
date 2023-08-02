import os
import time

from auv.mission import path_mission
from auv.utils import arm, deviceHelper

# load sub config
config = deviceHelper.variables

# arm.arm()

# create the mission object
pathMission = path_mission.PathMission(**config)

# run the mission
pathMission.run()

# terminate the mission
pathMission.cleanup()

# end
print("[INFO] Mission ended")
