"""
To create a sequential order of missions for Graey to follow
"""

import time 
import signal # For handling signals

from auv.mission import gate_mission, cointoss_mission, surfacing_mission, dhd_approach_mission # Various missions
from auv.utils import arm, disarm, deviceHelper # Loading, arming, disarming 
from auv.motion import robot_control # Control the motion of Graey
from auv.device.modems import modems_api # Modem code for intersub communication
import rospy

myNode = rospy.init_node("missions", anonymous=True) 
octagon_heading = 350 # Sets heading for Octagon mission relative to the gates
time.sleep(30)

# Load the sub configuration
config = deviceHelper.variables
# Load instance of the RobotControl class - class to handle movement of sub
rc = robot_control.RobotControl()

# Attempt to start intersub communication (modem)
try:
    modem = modems_api.Modem()
    modem.send_msg("graey handshake")
    fail_modem = False
except:
    fail_modem = True
    print("Failed to start modem, starting directly")

# For cleaning up modems and LEDs
def onExit(signum, frame):
    try:
        print("\Closing and exiting...")

        # Clean up modems and LEDs
        modems_api.led.clean()
        if not fail_modem:
            modem.stop()

        myNode.stop()
        time.sleep(3)
        rospy.signal_shutdown("Rospy Exited")
        while not rospy.is_shutdown():
            pass
        print("\n\nCleanly Exited")
        exit(1)
    except:
        pass

# In the event of a keyboard exception (Ctrl + C) launch the onExit function
signal.signal(signal.SIGINT, onExit)

# Arm the sub
arm.arm()

# Set the depth of the sub to 0.75 m
rc.set_depth(0.75)

# Start up Graey and set headings
if not fail_modem:
    modem.send_msg("start graey")
    modem.send_msg("set heading")

# Run the Gate Mission, passing through the "Earth" side
time.sleep(2)
rc.forwardDist(3.5, 2)
target = "earth"
gateMission = gate_mission.GateMission(target, **config)
gateMission.run()
gateMission.cleanup()

# Tell that the heading has ended
if not fail_modem:
    modem.send_msg("set heading end")
    modem.send_msg("traveling")

# Set the heading to move towards the octagon
rc.setHeadingOld(octagon_heading)
t = 0
while t < 70:  # For the next 70 seconds, move at power 2 forward # Need to adjust for finals
    rc.movement(forward=2)
    time.sleep(0.1)
    t += 0.1

# Reset time so that in the next second the thrusters are stopped (inertia dictates that the sub will continue moving forward)
t = 0
while t < 1:
    rc.movement(forward=0)
    time.sleep(0.1)
    t += 0.1

# Move to the DHD Approach mission
if not fail_modem:
    modem.send_msg("dhd approach")

# Run the DHD Approach mission (approach the Octagon)
dhd_app = dhd_approach_mission.DHDApproachMission(**config)
time.sleep(2)
dhd_app.run()
dhd_app.cleanup()

if not fail_modem:
    modem.send_msg("dhd approach end")
    modem.send_msg("surfacing")

# Run the surfacing mission
surfacing = surfacing_mission.SurfacingMission(**config)
time.sleep(2)
surfacing.run()
surfacing.cleanup()

if not fail_modem:
    modem.send_msg("surfacing end")

disarm.disarm()  # Disarm the sub so nothing bad happens
