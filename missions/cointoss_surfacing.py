import time

from auv.mission import cointoss_mission, surfacing_mission, dhd_approach_mission
from auv.utils import arm, disarm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# load sub config
config = deviceHelper.variables
arm.arm()

# Run coin toss
coin_toss = cointoss_mission.CoinTossMission()
time.sleep(2)
coin_toss.run(218) # NOTE: TWEAK THIS BEFORE MISSION
coin_toss.cleanup()

# Run dhd approach
dhd_app = dhd_approach_mission.DHDApproachMission()
time.sleep(2)
dhd_app.run()
dhd_app.cleanup()

# Run surfacing
surfacing = surfacing_mission.SurfacingMission()
time.sleep(2)
surfacing.run()
surfacing.cleanup()

disarm.disarm() # just in case