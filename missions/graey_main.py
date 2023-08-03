import time

from auv.mission import cointoss_mission, surfacing_mission, dhd_approach_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
import rospy

rospy.init_node("missions", anonymous=True)

time.sleep(30)

# load sub config
rc = robot_control.RobotControl()
config = deviceHelper.variables
arm.arm()

# Run coin toss
coin_toss = cointoss_mission.CoinTossMission()
time.sleep(2)
coin_toss.run(218) # NOTE: TWEAK THIS BEFORE MISSION
coin_toss.cleanup()

t=0
while(t < 50):
    rc.movement(forward=2)
    time.sleep(0.1)
    t+=0.1
    
t=0
while(t<1):
    rc.movement(forward=0)
    time.sleep(0.1)
    t+=0.1

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