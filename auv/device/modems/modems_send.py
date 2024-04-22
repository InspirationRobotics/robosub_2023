"""
Allows the user to send and receive responses through the modem. Essentially acts as a simple terminal for the modem.
"""

import time
import serial # Serial connection
import platform
from ...utils.deviceHelper import dataFromConfig # Configuration of the devices (in this case the modem)

modemsPort = dataFromConfig("modem")
print(modemsPort)

# Configure the serial connections
ser = serial.Serial(port=modemsPort, baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

ser.isOpen()

print('Enter your commands below.\r\nInsert "exit" to leave the application.')

_input = ""
while 1:
    _input = input(">> ")
    if _input == "exit":
        ser.close()
        exit()
    else:
        buffer = f'{_input}\r\n'
        print(buffer)
        ser.write(buffer.encode())
        out = b""
        time.sleep(1)
        while ser.inWaiting() > 0:
            out += ser.read(1)

        if out != b"":
            print(out.decode())
