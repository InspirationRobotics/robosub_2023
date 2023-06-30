import os

def setPwm(channel, pwm):
	os.system("cd /home/inspiration/auv/maestro-linux && ./UscCmd --servo " + str(channel) + "," + str(int(pwm*4)))