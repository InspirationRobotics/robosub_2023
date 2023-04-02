from navigation.rc import RCLib
import time
import navigation.imu as imu


rc = RCLib()

rc.setmode('ALT_HOLD')
#rc.setmode('MANUAL')

rc.arm()

rc.throttle("time",1, -0.40)
rc.throttle("time",1, -0.40)

rc.forward("time", 3, 0.30)
rc.forward("time", 3, -0.30)
rc.forward("time", 3, 0.30)
rc.forward("time", 3, -0.30)

rc.forward("time", 2, 0.40)
rc.yaw("imu", -45, 0.5)
rc.forward("time", 2, 0.40)
rc.yaw("imu", -45, 0.5)
rc.forward("time", 2, 0.40)
rc.yaw("imu", -45, 0.5)
rc.forward("time", 2, 0.40)

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

rc.disarm()
rc.close()
