# Import mavutil
import time
import imu
from pymavlink import mavutil
import math
from ac import *
from log import * 

# Create the connection

class RCLib:

    def __init__(self, logger=None, ac=None):
        try:
            if logger == None:
               logger = log.LogLib()
 
            self.log = logger

            self.master = mavutil.mavlink_connection(
                '/dev/ttyACM0',
                baud=115200)

            self.master.wait_heartbeat()

            if ac != None:
                self.ac = ACLib()
  

        except Exception as e:
            self.log.critical(e)
        
    # This function is responsible for sending RC channel overrides
    def set_rc_channel_pwm(self, id, pwm=1500):
        
        if (id < 1) or (id > 6):
            self.log.error('Channel does not exist' % id)
            return
    
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        if id < 9:
            
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,                # target_system
                self.master.target_component,             # target_component
                *rc_channel_values)

    def raw (self, id, pwm=1900) :
	# have to give a pwd value between 1100 and 1900
	if(pwm>1900):
		pwm = 1900
	if(pwm<1100):
		pwm = 1100
      try:

        if (id == "pitch") :

            self.set_rc_channel_pwm(1, pwm) 
            
        if (id == "roll") :

            self.set_rc_channel_pwm(2, pwm)

        if (id == "throttle") : # set the channel to a movement since pixhawk does not allow for individual thruster control

            self.set_rc_channel_pwm(3, pwm)

        if (id == "yaw") :

            self.set_rc_channel_pwm(4, pwm)

        if (id == "forward") :

            self.set_rc_channel_pwm(5, pwm)

        if (id == "lateral") :

            self.set_rc_channel_pwm(6, pwm)
      except:
        self.log.critical ('exception in raw')
        raise

    def replay2 (self):
        table = {}
        i = 0
        with open('p1.txt', 'r') as file :
          for line in file :
            v = line.split(',')
            table[i] = v
            i = i + 1
        for entry in table:
            cmds = table[entry]
            self.master.mav.rc_channels_override_send(
                self.master.target_system,          
                self.master.target_component,
                int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3]), int(cmds[4]), int(cmds[5]), int(cmds[6]), int(cmds[7]))
            #print int(cmds[0]), int(cmds[1]), int(cmds[2]), int(cmds[3]), int(cmds[4]), int(cmds[5]), int(cmds[6]), int(cmds[7])
            time.sleep(0.5)

    def arm (self) :
    
        self.master.arducopter_arm()

    def disarm (self) :

        self.master.arducopter_disarm()
        
    def throttle (self, unit, value, power) :
	''' 
	used in the seaperch autonomous to control depth but is not working properly
		when putting 1-2 seconds does not change the depth, when putting 4 seconds (positive and negative 4) made the sub try to go up out of the water
	'''
	pwm = 1500 + (400 * power) # is this what we have to do to power values to convert to correct amount of pwm value?
		
	if unit == "time" :

	    runtime = time.time() + value
	    while runtime > time.time() :
	        self.raw("throttle", pwm)
            self.raw("throttle", 1500)

    def forwardAngle (self, unit, value, power, angle) :
        
	if unit == "time" :
            
	    self.setmode('ALT_HOLD')
	    pwm = 1500 + (400 * power)
	    runtime = time.time() + value
            while runtime > time.time() :
                self.raw("forward", pwm)
                self.log.debug('forwardAngle delta %d' % runtime - time.time())
    
            self.raw("forward", 1500)
    	    #self.setmode('MANUAL')
 
		
    def forward (self, unit, value, power) :
        ''' 
	used for forward and backwards lateral movement, to move backward use negative power 
	'''
	if unit == "time" :
            
	    self.setmode('ALT_HOLD')
	    pwm = 1500 + (400 * power)
	    runtime = time.time() + value
            while runtime > time.time() :
                self.raw("forward", pwm)
                #self.log.info(runtime - time.time())

            self.raw("forward", 1500)
    	    #self.setmode('MANUAL')
    
    def deg (self) :

        self.log.info(imu.getDeg(self.master)) 

    def yaw (self, unit, value, power=None) :
	''' 
	used for turning left and right  
	'''
        power = power * (value/abs(value))
    
    
        if unit == "time" :
   
            pwm = 1500 + (400 * power)
            runtime = time.time() + value
            while runtime > time.time() :
    
                self.raw("yaw", pwm)
    
            self.raw("yaw", 1500)
    
        if unit == "imu" :
	    # when using imu and setting the value to 45 it turns 90 degrees instead of 45, positive values turn robot to the right, negative values turn robot to the left

            start = imu.getDeg(self.master)
            self.log.info('start angle: %s' % start)

            end = start + value
            offset = 0
            flag = 0
            
            if value > 0 :

                if (end > 360) :

                    offset = 360
                    
                #self.raw("yaw", pwm)
                while imu.getDeg(self.master) + (offset * flag) < end:
                    self.log.debug('Loop 1')
                    if power == None:
                        pwm = 1500 + (end - (imu.getDeg(self.master) + (offset * flag)))*0.5 + 55
                    else :
                        pwm = 1500 + (400 * power)

                    self.log.debug ('pwm = %d') % pwm

                    if imu.getDeg(self.master) < (start - 10):
                        flag = 1

                    self.raw("yaw", pwm)
                    self.log.debug(imu.getDeg(self.master))
                    self.log.debug(time.time())
                self.raw("yaw", 1500)
    
            if value < 0 :

                if (end < 0) :

                    offset = -360

                #self.raw("yaw", pwm)
                while imu.getDeg(self.master) + (offset * flag) > end:

                    if power == None:
                        pwm = 1500 - ((imu.getDeg(self.master) + (offset * flag)) - end)*0.5 - 55
                    else :
                        pwm = 1500 + (400 * power)

                    if imu.getDeg(self.master) > (start + 10):
                        flag = 1

                    self.raw("yaw", pwm)
                    self.log.debug(imu.getDeg(self.master))

                self.raw("yaw", 1500)

            self.log.debug('before delay')
	    self.log.debug(imu.getDeg(self.master))
            #r = time.time() + 20
	    #while (r > time.time()):
		#self.log.debug(imu.getDeg(self.master)) 

            #time.sleep(10) 
	    self.log.debug('after delay')
	    self.log.debug(imu.getDeg(self.master))
            
            self.log.info('Expected End angle: %s' % end)
            self.log.info('Actual End angle: %s' % imu.getDeg(self.master))

    def getDeg (self) :
        r = imu.getDeg(self.master)
        self.log.debug(r)
        return imu.getDeg(self.master) 
                  
    def lateral (self, unit, value, power) :

        if unit == "time" :

            if value < 0:
                power = power * -1
                value = value * -1

            self.setmode('ALT_HOLD')
            pwm = 1500 + (400 * power)
            runtime = time.time() + value
            while runtime > time.time() :

                self.raw("lateral", pwm)
                self.log.debug(runtime - time.time())

            self.raw("lateral", 1500)
            #self.setmode('MANUAL')
   
    def lateralDist (self, value ) :
            time = value/200
            self.lateral("time", time, 0.2)

    def close (self) :
        self.master.close()

    def killall (self) :
    
        self.set_rc_channel_pwm(1, 1500)
        self.set_rc_channel_pwm(2, 1500)
        self.set_rc_channel_pwm(3, 1500)
        self.set_rc_channel_pwm(4, 1500)
        self.set_rc_channel_pwm(5, 1500)
        self.set_rc_channel_pwm(6, 1500)
        
    def setmode (self, mode_val) :
    
        mode = mode_val
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
        
    def angleWrap (self, angle) :
        
        if angle > 360 :
            angle = angle % 360

        if angle < 0 :
            angle = angle + 360

        return angle

    def imu_turn (self, angle):

        #TURN_THRESHOLD = 3
        TURN_THRESHOLD = 3
        self.setmode('ALT_HOLD')
        #error = angle - self.getDeg()
        angle = self.angleWrap(angle)
        
        self.log.info(self.getDeg())
        self.log.info("target = ", angle)
        
        while (abs(self.getError(angle)) > TURN_THRESHOLD):
            
            pwm = self.getSteer(self.getError(angle))
            self.log.info('speed: ', pwm)
            self.raw('yaw', pwm)
            
        self.raw('yaw', 1500)

                
    def getError(self, angle):
        error = angle - self.getDeg()
        return error

    def getSteer (self, error):
        kP = 0.003
        #kP = 0.05
        end_speed = abs(kP*error)
        #final_speed = np.clip(end_speed, 0.1, 1)
        #converted_speed = 1500 + (end_speed*400)
        if (end_speed > 0.35):
            final_speed = 0.35
        elif (end_speed < 0.1375):
            final_speed = 0.1375
        else:
            final_speed = end_speed


        if (error > 0):
            #turn right
            return_speed = 1500 + (400*final_speed)
        else:
            return_speed = 1500 - (400*final_speed)
            
            
        return return_speed

    def move_dist(self, distance_in, speed):
        DISTANCE_CONSTANT = 48.181818
        time = (distance_in)/(speed*DISTANCE_CONSTANT)
        self.forward('time', time, speed)

    def test(self):
        self.log.info('Test from RC')

    def gateAlgin (self) :

        x1 = 0
        x2 = 0
        y1 = 0
        y2 = 0
