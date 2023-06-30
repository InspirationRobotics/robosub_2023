import os
import time

def setPwm(channel, pwm):
	os.system("cd /home/inspiration/auv/maestro-linux && ./UscCmd --servo " + str(channel) + "," + str(int(pwm*4)))

#torpedo channel 2:
#load: 2400
#Torpedo 1: 1700 (1800 actual)
#Torpedo 2: 1300 (1400 actual
# 
#marker dropper channel 1:
#Load: 1600
#Ball 1: 1200
#Ball 2: 700
#
# gripper channel 0:
# no incremental control; just open and close
# open: 1550 1.5 seconds then 1500 to stop
# close: 1450 1.5 seconds then 1500 to stop

def dropper(ball):
	if ball==0:
		setPwm(1, 1600)
	elif ball==1:
		setPwm(1,1200)
	elif ball==2:
		dropper(1)
		time.sleep(0.5)
		setPwm(1,700)
		time.sleep(1)
		dropper(0)

dropper(2)