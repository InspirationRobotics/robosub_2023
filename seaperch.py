from navigation.rc import RCLib # in the 2019 robosub repo called inspiration_robosub/navigation/
import time
import navigation.imu as imu # in the 2019 robosub repo called inspiration_robosub/navigation/


rc = RCLib()

rc.setmode('ALT_HOLD') # is this is running in the background to keep the robot steady? 
#rc.setmode('MANUAL')

rc.arm()

rc.throttle("time",1, -0.40) # this is to control depth, was going up instead of down
rc.throttle("time",1, -0.40) # inside the parentheses set the meaurement to time, the time value in seconds, and the power

# moving forwards and backwards
rc.forward("time", 3, 0.30)
rc.forward("time", 3, -0.30)
rc.forward("time", 3, 0.30)
rc.forward("time", 3, -0.30)

# moving in a square
rc.forward("time", 2, 0.40)
rc.yaw("imu", -45, 0.5) # this turns 90 degrees instead of 45
rc.forward("time", 2, 0.40)
rc.yaw("imu", -45, 0.5)
rc.forward("time", 2, 0.40)
rc.yaw("imu", -45, 0.5)
rc.forward("time", 2, 0.40)

# spinning in a circle with incremented turns, then spin in opposite direction
rc.yaw("imu", 45, 0.8)
rc.yaw("imu", 45, 0.8)
rc.yaw("imu", 45, 0.8)
rc.yaw("imu", 45, 0.8)
rc.yaw("imu", 45, 0.8)
rc.yaw("imu", 45, 0.8)
rc.yaw("imu", 45, 0.8)
rc.yaw("imu", -45, 0.8)
rc.yaw("imu", -45, 0.8)
rc.yaw("imu", -45, 0.8)
rc.yaw("imu", -45, 0.8)
rc.yaw("imu", -45, 0.8)
rc.yaw("imu", -45, 0.8)
rc.yaw("imu", -45, 0.8)

# try and add straffing sidways and diagonally to see if those work with this configuration 

rc.disarm()
rc.close()
