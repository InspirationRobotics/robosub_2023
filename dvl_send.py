import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
	port='/dev/cu.usbserial-1240',
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS
)

#ser.close()
#ser.open()
ser.isOpen()

print('Enter your commands below.\r\nInsert "exit" to leave the application.')

_input=''
while 1 :
	# get keyboard input
	#input = raw_input(">> ")
        # Python 3 users
	_input = input(">> ")
	if (_input == 'exit'):
		ser.close()
		exit()
	else:
		# send the character to the device
		# (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
		buffer = _input + '\r\n'
		print(buffer)
		ser.write(buffer.encode())
		out = b''
		# let's wait one second before reading output (let's give device time to answer)
		time.sleep(1)
		while ser.inWaiting() > 0:
			out += ser.read(1)
			
		if out != b'':
			print(out)
