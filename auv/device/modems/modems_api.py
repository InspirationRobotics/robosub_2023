import time
import serial
import threading
from ...utils.deviceHelper import dataFromConfig

modemsPort = dataFromConfig("modem")

# configure the serial connections
ser = serial.Serial(
    port=modemsPort,
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
)

ser.isOpen()


class Modem:
    def __init__(self, targetModemAddr=None):
        self.modemAddr = None
        self.voltage = None
        self.receiveActive = False
        self.targetModemAddr = targetModemAddr
        self.queryStatus()
        self.thread_param_updater = threading.Timer(0, self.receiveLoop)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()

    def sendToModem(self, data):
        data = f'${data}\r\n'
        ser.write(data.encode())
        out = b""
        time.sleep(0.1)
        while ser.inWaiting() > 0:
            out += ser.read(1)
        if out != b"":
            try:
                rawOut = out.decode("utf-8").replace("\n", "").replace("\r", "")
                # print(rawOut)
                return rawOut
            except:
                print("Failed to parse")

    def queryStatus(self):
        QUERY_COMMAND = "?"
        data = self.sendToModem(QUERY_COMMAND)
        self.modemAddr = data[2:5]
        self.voltage = round(float(int(data[6:11]) * 15 / 65536), 2)
        print(f"Modem Addr: {self.modemAddr}")
        print(f"Voltage: {str(self.voltage)}")

    def transmitDataLowLevel(self, msg, broadcast=True):
        length = len(msg.encode("utf-8"))
        if length > 64:
            print(f"Packet size of {length} is too large, limited to 64 bytes")
            return
        if length < 10:
            length = f"0{str(length)}"
        else:
            length = str(length)
        if broadcast:
            data = f"B{length}{msg}"
            self.sendToModem(data)
        else:
            data = f"U{self.targetModemAddr}{length}{msg}"
            self.sendToModem(data)

    def transmit(self, msg, modemAddr=None, broadcast=True):
        if modemAddr != None:
            self.targetModemAddr = modemAddr
            broadcast = False
        msg = f"*{msg}*"
        length = len(msg.encode("utf-8"))
        if length > 64:
            if length % 64 < 3:
                msg = f"  {msg}"
                length += 2
        dataToSend = []
        while length > 64:
            dataToSend.insert(0, msg[-64:])
            length -= 64
            msg = msg[:length]
        if not length <= 0:
            dataToSend.insert(0, msg)
        print(dataToSend)
        for data in dataToSend:
            self.transmitDataLowLevel(data, broadcast)
            sleepAmt = round(0.205 + (len(msg.encode("utf-8")) + 16) * 0.0125, 3)
            time.sleep(sleepAmt)

    def receiveLoop(self):
        data = ""
        state = False
        while True:
            while self.receiveActive:
                if ser.inWaiting() > 0:
                    out = b""
                    out = ser.read(1)
                    if out != b"":
                        try:
                            rawOut = out.decode("utf-8")
                            if rawOut == "*":
                                state = not state
                                data += rawOut
                            if state:
                                data += rawOut
                        except:
                            pass
                if not state and len(data) > 0:
                    print(data)
                    data = ""

    def startReceive(self):
        self.receiveActive = True

    def stopReceive(self):
        self.receiveActive = False


# modem = Modem()

# modem.transmit("Hello World!")
# modem.startReceive()
# while True:
#    pass
