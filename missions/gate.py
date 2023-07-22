import os
import time

from auv.mission import gate_mission
from auv.utils import arm, deviceHelper

# load sub config
config = deviceHelper.variables

#arm.arm()
time.sleep(5)

# create the mission object
gateMission = gate_mission.GateMission(**config)

# run the mission
gateMission.run()

# terminate the mission
gateMission.cleanup()

# end
print("[INFO] Mission ended")
