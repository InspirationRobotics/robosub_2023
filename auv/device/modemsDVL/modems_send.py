import time
import serial
import platform
from ...utils.deviceHelper import findDevice


if("nx" in platform.node()):
    id = "platform-3610000.xhci-usb-0:2.3.4:1.0"
else:
    id = "platform-70090000.xusb-usb-0:2.1:1.0"

modemsPort = findDevice(id)

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
