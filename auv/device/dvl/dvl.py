import signal
import time

import serial

from ...utils.deviceHelper import dataFromConfig

class DVL:
    """DVL class"""
    def __init__(self):
        self.dvlPort = dataFromConfig("dvl")
        
        self.ser = serial.Serial(
            port=self.dvlPort,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        self.ser.isOpen()
        self.ser.reset_input_buffer()
        self.ser.send_break()
        time.sleep(1)
        startPing = "CS"
        self.ser.write(startPing.encode())
        time.sleep(2)
    pass