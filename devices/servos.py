import os
import time
import serial

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

global gripState
gripState = True #true for open, false for closed
class servos:

	def __init__(self):
		self.torpedoLauncher(0)
		self.dropper(0)
		print("Initialized...")


	def setPwm(self, channel, pwm): 
		#if there is an error with this function open noMachine into jetson 
		#and go to auv/maestro-linux folder and do ./MaestroControlCenter and go to errors tab and then clear the errors
		#you can then close it and try running these functions again and it should work
		os.system("cd /home/inspiration/auv/maestro-linux && ./UscCmd --servo " + str(channel) + "," + str(int(pwm*4)))

	#please test this code for gripper, the 1.5 seconds will likely need to be adjusted (should be close though)
	#if gripper keeps pushing after reaching its limit, it WILL KILL our power distribution board
	def gripper(self, state): # true for to open, false for to close
		global gripState
		if(gripState==state):
			return
		pwm=1500
		if(state):
			pwm = 1550
		else:
			pwm = 1450
		startTime = time.time()
		while(time.time()-startTime<1.5): #run for 1.5 seconds
			self.setPwm(0,pwm)
			time.sleep(0.05)
		self.setPwm(0,1500) #stops gripper
		self.setPwm(0,1500) #redundant
		gripState = state

	def torpedoLauncher(self, torpedo): #0,1,2 : load, fire 1, fire 2
		if torpedo == 0: #load state
			self.setPwm(2, 2400)
		elif torpedo == 1: #fire torpedo 1
			self.setPwm(2, 1700)
		elif torpedo == 2: #fire torpedo 2
			self.torpedoLauncher(1) #incase first has not been fired yet
			time.sleep(0.5)
			self.setPwm(2, 1300)
			time.sleep(1)
			self.torpedoLauncher(0) #reset to load position now that both are fired
		else:
			print("Invalid Arg in [torpedoLauncher]")

	def dropper(self, ball): #0,1,2 : load, drop 1, drop 2
		if ball==0: #load balls
			self.setPwm(1, 1600)
		elif ball==1: #Drop first ball
			self.setPwm(1,1200) 
		elif ball==2: #Drop second ball
			self.dropper(1) #incase dropper(1) is not called before allows drop of both balls without collision
			time.sleep(0.5)
			self.setPwm(1,700)
			time.sleep(1)
			self.dropper(0) #resets to load position now that its empty
		else:
			print("Invalid Arg in [dropper]")
