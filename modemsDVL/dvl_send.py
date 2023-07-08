import time
import serial

#to send commands to DVL based off documentation, type "break" first
#to start pings send "CS"

# configure the serial params for the DVL
ser = serial.Serial(
	port='/dev/ttyUSB1', #platform-3610000.xhci-usb-0:2.3.1:1.0
	baudrate=115200, #default is 9600 but DVL is set to 115200
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
	_input = input(">> ")
	ser.reset_input_buffer()
	if (_input == 'exit'):
		ser.close()
		exit()
	else:
		buffer = _input + '\r'
		if(buffer=="break\r"):
			ser.send_break()
			continue
		print(buffer)
		ser.write(buffer.encode())
		out = b''
		time.sleep(5)
		while ser.inWaiting() > 0:
			out += ser.read(1)
		if out != b'':
			try:
				rawOut = out.decode("utf-8")
				#out = rawOut[rawOut.find(_input):rawOut.rfind(">")]
				print(rawOut)
			except:
				print("Failed to parse")
