import time
import serial
import os

modemsPort = os.popen('python3 /home/inspiration/auv/scripts/deviceHelper.py platform-3610000.xhci-usb-0:2.3.4:1.0').read().replace("\n", "")

# configure the serial connections
ser = serial.Serial(
	port=modemsPort,
	baudrate=9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS
)

ser.isOpen()

print('Enter your commands below.\r\nInsert "exit" to leave the application.')

_input=''
while 1 :
	_input = input(">> ")
	if (_input == 'exit'):
		ser.close()
		exit()
	else:
		buffer = _input + '\r\n'
		print(buffer)
		ser.write(buffer.encode())
		out = b''
		time.sleep(1)
		while ser.inWaiting() > 0:
			out += ser.read(1)
			
		if out != b'':
			print(out.decode())
