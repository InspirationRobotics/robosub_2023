import time
import serial
import signal
from ....utils.deviceHelper import dataFromConfig

dvlPort = dataFromConfig("dvl")

ser = serial.Serial(
	port=dvlPort,
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS
)

ser.isOpen()
ser.reset_input_buffer()
ser.send_break()
time.sleep(1)
startPing = "CS"
ser.write(startPing.encode())
time.sleep(2)

def createPacket(SA):
	dataPacket = {
		"Timestamp":[],             # year, month, day, hour:minute:second
		"TimestampSeconds": [],		# timestamp in seconds (ignores year, month, day)
		"Attitude":[],              # roll, pitch, and heading in degrees
		"Salinity":[],              # in ppt (parts per thousand)
		"Temp":[],                  # celcius
		"Transducer_depth":[],      # meters
		"Speed_of_sound":[],        # meters per second
		"Result_code": [],
		"DVL_velocity":[],          # mm/s # xyz error
		"isDVL_velocity_valid": [],   # boolean
		"AUV_velocity":[],          # mm/s # xyz
		"isAUV_velocity_valid": [],   # boolean
		"Distance_from_bottom": [], # meters
		"Time_since_valid": []      # seconds
	}
	SA = SA.decode("utf-8").replace(" ", "").replace("\r\n", "").split(",")
	if(SA[0]==":SA"):
		dataPacket["Attitude"] = [float(SA[1]), float(SA[2]), float(SA[3])]
		TS = ser.readline().decode("utf-8").replace(" ", "").replace("\r\n", "").split(",")
		dataPacket["Timestamp"] = [f"20{TS[1][:2]}-{TS[1][4:6]}-{TS[1][2:4]}", f"{TS[1][6:8]}:{TS[1][8:10]}:{TS[1][10:12]}"]
		dataPacket["TimestampSeconds"] = [int(TS[1][6:8])*60*60+int(TS[1][8:10])*60+int(TS[1][10:12])+int(TS[1][12:14])*0.01] 
		dataPacket["Salinity"] = float(TS[2])
		dataPacket["Temp"] = float(TS[3])
		dataPacket["Transducer_depth"] = float(TS[4])
		dataPacket["Speed_of_sound"] = float(TS[5])
		dataPacket["Result_code"] = TS[6]
		BI = ser.readline().decode("utf-8").replace(" ", "").replace("\r\n", "").split(",")
		dataPacket["DVL_velocity"] = [int(BI[1]), int(BI[2]), int(BI[3]), int(BI[4])]
		dataPacket["isDVL_velocity_valid"] = (BI[5]=="A")
		BS = ser.readline().decode("utf-8").replace(" ", "").replace("\r\n", "").split(",")
		dataPacket["AUV_velocity"] = [int(BS[1]), int(BS[2]), int(BS[3])]
		dataPacket["isAUV_velocity_valid"] = (BS[4]=="A")
		BE = ser.readline().decode("utf-8").replace(" ", "").replace("\r\n", "").split(",") #unused
		BD = ser.readline().decode("utf-8").replace(" ", "").replace("\r\n", "").split(",")
		dataPacket["Distance_from_bottom"] = float(BD[4])
		dataPacket["Time_since_valid"] = float(BD[5])
		return dataPacket



if __name__ == '__main__':
	state = True
	while (state):
		out = b''
		out = ser.readline()
		if out != b'':
			try:
				rawOut = out.decode("utf-8")
				packet = createPacket(out)
				print(packet)
				if(packet["isAUV_velocity_valid"]==True):
					print(packet["AUV_velocity"])
			except:
				pass

	def onExit(signum, frame):
		try:
			state = False
			ser.close()
			time.sleep(1)
			exit(1)
		except:
			pass

	signal.signal(signal.SIGINT, onExit)
