from auv.mission import bin_mission
from auv.utils import arm, deviceHelper

# load sub config
config = deviceHelper.variables

# make sure the sub is armed
arm.arm() 
time.sleep(2)

# create the mission object
binMission = bin_mission.BinMission(**config)

# run the mission
binMission.run()

# terminate the mission
binMission.cleanup()

# end
print("[INFO] Mission ended")