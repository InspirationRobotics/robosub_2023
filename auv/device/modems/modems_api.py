import time
import serial
import threading
import math
from ...utils.deviceHelper import dataFromConfig


class LED:
    def __init__(self, port):
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        assert self.ser.isOpen(), "Failed to open serial port"

    def on_send_msg(self):
        self.ser.write("t".encode())

    def on_recv_msg(self):
        self.ser.write("r".encode())

class DummyLED:
    def __init__(self, port):
        pass

    def on_send_msg(self):
        pass

    def on_recv_msg(self):
        pass

class Modem:
    def __init__(self, on_receive_msg=None, auto_start=True):
        port = dataFromConfig("modem")
        self.ser = serial.Serial(
            port=port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        assert self.ser.isOpen(), "Failed to open serial port"

        if on_receive_msg is not None:
            self.on_receive_msg = on_receive_msg

        # init LED
        try:
            self.led = LED(port)
        except:
            self.led = DummyLED(port)
            print("Failed to init LED")

        # ACK is used to ensure message is received
        self.ACK = 0
        self.blocked = False
        self.in_transit = []  # [message, time_sent, ack-expected, modemaddr, priority]
        self.ack_received = []

        self.modemAddr = None
        self.voltage = None
        self.query_status()

        self.receive_active = True
        self.sending_active = True

        self.thread_recv = threading.Thread(target=self._receive_loop)
        self.thread_send = threading.Thread(target=self._send_loop)

        if auto_start:
            self.start()

    def _send_to_modem(self, data):
        data = f"${data}\r\n"
        self.ser.write(data.encode())
        out = b""
        time.sleep(0.1)
        while self.ser.inWaiting() > 0:
            out += self.ser.read(1)
        if out != b"":
            try:
                rawOut = out.decode("utf-8").replace("\n", "").replace("\r", "")
                # print(rawOut)
                return rawOut
            except:
                print("Failed to parse")

    def query_status(self):
        QUERY_COMMAND = "?"
        data = self._send_to_modem(QUERY_COMMAND)
        self.modemAddr = data[2:5]
        self.voltage = round(float(int(data[6:11]) * 15 / 65536), 2)
        print(f"Modem Addr: {self.modemAddr}")
        print(f"Voltage: {str(self.voltage)}")

    def _transmit_data_low_level(self, msg, dest_addr, broadcast):
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
            self._send_to_modem(data)
        else:
            data = f"U{dest_addr}{length}{msg}"
            self._send_to_modem(data)

    def _transmit(self, msg, ack=None, dest_addr=None, broadcast=True):
        """Transmit the msg to the modem, blocking until the message is sent"""
        while self.blocked:
            time.sleep(0.01)
        self.blocked = True

        if dest_addr is not None:
            broadcast = False

        if msg is None:
            # ack message
            msg = f"@{ack}@"
        else:
            # normal message with ack
            # if ack is None, generate a new ack
            if ack is None:
                self.ACK += 1
                ack = self.ACK
            msg = f"*{msg}*@{ack}@"

        # max length is 64 bytes
        # min length is 3 bytes
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
        if length > 0:
            dataToSend.insert(0, msg)

        for data in dataToSend:
            self.led.on_send_msg()
            self._transmit_data_low_level(data, dest_addr, broadcast)
            sleepAmt = round(0.205 + (len(msg.encode("utf-8")) + 16) * 0.0125, 3)
            time.sleep(sleepAmt)

        self.blocked = False
        return time.time()

    def send_msg(self, msg, ack=None, dest_addr=None, priority=0):
        """
        Append a msg to the sending list, non-blocking
        The message would be processed after all the previous messages were sent out

        messages with priority 0 will be timed out after 30 seconds
        messages with priority 1 won't be timed out
        """
        if ack is None:
            self.ACK += 1
            ack = self.ACK

        self.in_transit.append([msg, time.time(), 0, ack, dest_addr, priority])

    def send_ack(self, ack, dest_addr=None):
        """
        Send an ack to the modem, non blocking
        this will only send an ACK, not a message
        """
        packet = [None, time.time(), 0, ack, dest_addr, 0]
        self.in_transit.insert(0, packet)

    def _send_loop(self):
        while self.sending_active:
            to_remove = []
            self.ack_received = []

            for it, packet in enumerate(self.in_transit):
                msg, time_sent, time_last_sent, ack, dest_addr, priority = packet

                if msg is None:
                    self._transmit(msg, ack=ack, dest_addr=dest_addr)
                    to_remove.append(it)
                    continue

                if time.time() - time_sent > 30 and priority == 0:
                    print(f'[WARNING] Message "{msg}" timed out')
                    to_remove.append(it)
                    continue

                # retry if no ack received after 1 seconds (better be safe than sorry)
                if time.time() - time_last_sent > 1.0 and ack not in self.ack_received:
                    ret = self._transmit(msg, ack=ack, dest_addr=dest_addr)
                    packet[2] = ret

                # print(self.ack_received)

            time.sleep(0.1)

            # remove timed out messages and received acks
            self.in_transit = [packet for it, packet in enumerate(self.in_transit) if it not in to_remove and packet[3] not in self.ack_received]

    def _receive_loop(self):
        data = ""
        expecting_ack = False
        msg_state = False  # True = message in receiving

        while self.receive_active:
            if self.ser.inWaiting() > 0:
                out = b""
                out = self.ser.read(1)
                if out != b"":
                    try:
                        rawOut = out.decode("utf-8")

                        # detect when we are changing state
                        if rawOut == "*" or rawOut == "@":
                            msg_state = not msg_state

                        if msg_state:
                            data += rawOut

                        # finished receiving message
                        if not msg_state and len(data) > 0:
                            # dispatch to msg callback
                            if "*" in data:
                                self.on_receive_msg(data)
                                expecting_ack = True
                                data = ""
                            # dispatch to ack callback
                            if "@" in data:
                                self.on_receive_ack(data, expecting_ack)
                                expecting_ack = False
                                data = ""

                    except KeyboardInterrupt:
                        break

                    except Exception as e:
                        print(e)
                        data = ""
                        pass

    def on_receive_msg(self, msg: str):
        # TODO: log into a file or use it for events triggers
        msg = msg.replace("*", "")
        print("Received message:", msg)

    def on_receive_ack(self, ack: str, expecting_ack):
        # just after a message (ack of the message)
        ack = ack.replace("@", "")
        ack = int(ack)

        # print(ack, self.in_transit, expecting_ack)
        if expecting_ack:
            self.send_ack(ack)
            return

        # ack of a message previously sent
        # remove the corresponding msg from in_transit
        for i, packet in enumerate(self.in_transit):
            if packet[3] == ack:
                self.ack_received.append(ack)
                return

        print(f'[WARNING] Received ack: "{ack}" but no corresponding message found, maybe timed out?')

    def start(self):
        self.receive_active = True
        self.sending_active = True
        self.thread_recv.start()
        self.thread_send.start()

    def stop(self):
        self.receive_active = False
        self.sending_active = False
        self.thread_recv.join()
        self.thread_send.join()


def dummy_callback(msg: str):
    print("Received message:", msg)


def on_receive_msg_logging(msg: str, log_file: str):
    self.led.on_recv_msg()
    msg = msg.replace("*", "")
    print("Received message:", msg)
    with open(log_file, "a+") as f:
        f.write(f"[{time.time()}]{msg}\n")


def manual_coms():
    modem = Modem()
    while True:
        msg = input("Enter message: ")
        modem.send_msg(msg, priority=1)


def handshake_start(self: Modem):
    """waits for a handshake to continue"""

    received_handshake = False

    def on_receive_msg(msg: str):
        if msg == "handshake":
            print("Handshake received")
            received_handshake = True
            self.on_receive_msg = dummy_callback

        try:
            on_receive_msg_logging(msg, "underwater_coms.log")
        except:
            print("[WARNING] Failed to log message [WARNING]")

    self.on_receive_msg = on_receive_msg

    # sending handshake (no timeout)
    msg = "handshake"
    self.send_msg(msg, ack=42, priority=0)
    print("Waiting for handshake...")

    while not received_handshake:
        if len(self.in_transit) == 0:
            break
        time.sleep(0.1)
    print("Handshake complete")


if __name__ == "__main__":
    modem = Modem()
    handshake_start(modem)
