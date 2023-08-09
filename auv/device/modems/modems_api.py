import Jetson.GPIO as GPIO
import time
import serial
import threading
from ...utils.deviceHelper import dataFromConfig, variables

port = dataFromConfig("modem")


class LED:
    def __init__(self):
        self.t_pin = 31
        self.r_pin = 32

    def on_send_msg(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.t_pin, GPIO.OUT)
        GPIO.output(self.t_pin, GPIO.HIGH)
        time.sleep(0.1)
        self.clean()

    def on_recv_msg(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.r_pin, GPIO.OUT)
        GPIO.output(self.r_pin, GPIO.HIGH)
        time.sleep(0.1)
        self.clean()

    def clean(self):
        # set to low to turn off
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.t_pin, GPIO.OUT)
        GPIO.setup(self.r_pin, GPIO.OUT)
        GPIO.output(self.t_pin, GPIO.LOW)
        GPIO.output(self.r_pin, GPIO.LOW)
        GPIO.cleanup()


led = LED()


class Modem:
    def __init__(self, auto_start=True):
        self.led = LED()
        self.ser = serial.Serial(
            port=port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        # callbacks for each msg type
        self.parse_msg = {
            "#B": self.parse_B,
            "#U": self.parse_U,
            "#R": self.parse_R,
            "#T": self.parse_TO,
        }
        self.recv_callbacks = [
            self.on_receive_flash_led,
            self.on_receive_msg_logging,
            self.on_receive_ack,
        ]
        self.send_callbacks = [
            self.on_send_msg_logging,
        ]
        self.data_buffer = ""  # stores previously received data to handle large messages

        # a simple ACK is used to ensure message is received
        self.ACK = 0
        self.blocked = False
        self.in_transit = []  # [message, time_sent, ack-expected, modemaddr, priority]
        self.ack_received = []

        # infos specific to this modem
        self.modemAddr = None
        self.voltage = None

        self.receive_active = True
        self.sending_active = True
        self.received_handshake = False

        self.thread_recv = threading.Thread(target=self._receive_loop)
        self.thread_send = threading.Thread(target=self._send_loop)

        if auto_start:
            self.start()

    def _send_to_modem(self, data):
        buffer = f"${data}"
        self.ser.write(buffer.encode())
        out = b""
        time.sleep(0.1)
        while self.ser.inWaiting() > 0:
            out += self.ser.read(1)
        if out != b"":
            output = out.decode()
            return output

    def query_status(self):
        QUERY_COMMAND = "?"
        data = self._send_to_modem(QUERY_COMMAND)
        if data is None:
            return
        self.modemAddr = data[2:5]
        self.voltage = round(float(int(data[6:11]) * 15 / 65536), 2)
        print(f"Modem Addr: {self.modemAddr}")
        print(f"Voltage: {str(self.voltage)}")

    def _transmit_packet(self, msg, addr=None):
        # select the transmission mode
        prefix = "U"
        if addr is None:
            prefix = "B"
            addr = ""

        # encode length
        length = len(msg.encode("utf-8"))
        if length < 10:
            length = f"0{str(length)}"

        # send the message
        data = f"{prefix}{addr}{length}{msg}"
        self._send_to_modem(data)

    def _transmit(self, msg, ack=None, dest_addr=None):
        """Transmit the msg to the modem, blocking until the message is sent"""
        while self.blocked:
            time.sleep(0.01)
        self.blocked = True

        if msg is None:
            # ack message
            msg = f"@{ack}"
        else:
            # normal message with ack
            # if ack is None, generate a new ack
            if ack is None:
                self.ACK += 1
                ack = self.ACK
            msg = f"*{msg}*@{ack}"

        # max length is 64 bytes
        # min length is 3 bytes
        length = len(msg.encode("utf-8"))
        if length > 64:
            if length % 64 < 3:
                msg = f"  {msg}"
                length += 2

        # split the message into multiple packets
        dataToSend = []
        while length > 64:
            dataToSend.insert(0, msg[-64:])
            length -= 64
            msg = msg[:length]
        if length > 0:
            dataToSend.insert(0, msg)

        for data in dataToSend:
            length = len(data.encode("utf-8"))

            self.led.on_send_msg()
            self._transmit_packet(data, dest_addr)

            sleepAmt = round(0.105 + (length + 16) * 0.0125, 3)
            time.sleep(sleepAmt)

        self.blocked = False
        t = time.time()

        # send callbacks
        for callback in self.send_callbacks:
            callback(dest_addr, msg, ack, None)

        return t

    def send_msg(self, msg, ack=None, dest_addr=None, priority=1):
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
        for it, p in enumerate(self.in_transit):
            if p[0] is not None or p[3] > ack:
                self.in_transit[it] = packet
                return
        self.in_transit.append(packet)

    def _send_loop(self):
        while self.sending_active:
            to_remove = []
            self.ack_received = []

            for it, packet in enumerate(self.in_transit):
                msg, time_sent, time_last_sent, ack, dest_addr, priority = packet

                # stop sending
                if self.sending_active is False:
                    return

                # send ACK
                if msg is None:
                    self._transmit(msg, ack=ack, dest_addr=dest_addr)
                    to_remove.append(it)
                    continue

                # timeout if no ack received after 30 seconds
                if time.time() - time_sent > 30 and priority == 0:
                    print(f'[WARNING] Message "{msg}" timed out')
                    to_remove.append(it)
                    continue

                # retry if no ack received after 1 seconds (better be safe than sorry)
                if time.time() - time_last_sent > 1.0 and ack not in self.ack_received:
                    ret = self._transmit(msg, ack=ack, dest_addr=dest_addr)
                    packet[2] = ret

                time.sleep(0.5)
            time.sleep(0.5)

            # remove timed out messages and received acks
            self.in_transit = [packet for it, packet in enumerate(self.in_transit) if it not in to_remove and packet[3] not in self.ack_received]

    def _dispatch(self, packet):
        """Each message is then dispatched to its corresponding callbacks"""
        msg_type = packet[:2]

        if msg_type not in self.recv_callbacks.keys():
            return

        # parse msg
        src_addr, msg, ack, distance = self.parse_msg[msg_type](packet)

        # determine if message is complete or not
        # NOTE: issue if we receive different messages from diff modems at the same time
        if msg is not None and msg[-1] != "*":
            self.data_buffer += msg
            return

        # end of message
        if msg is not None:
            msg = self.data_buffer + msg
            self.data_buffer = ""

        # dispatch to callbacks
        for callback in self.recv_callbacks[msg_type]:
            callback(src_addr, msg, ack, distance)

    def _receive_loop(self):
        while self.receive_active:
            # read a whole line
            raw_packet = self.ser.readline()
            if raw_packet == b"":
                pass

            try:
                # TODO: handle large messages
                packet = raw_packet.decode("utf-8")
                self._dispatch(packet)

            except KeyboardInterrupt:
                break

            except Exception as e:
                print(e)
                pass

    def parse_B(self, packet: str):
        """Broadcast ; packet format: #Bxxxnnddd"""

        msg_type = packet[:2]
        # verify that the message is a broadcast message
        if msg_type != "#B":
            print("Received a non broadcast message in broadcast callback")
            return

        src_addr = packet[2:5]
        length = int(packet[5:7])

        data = packet[7 : 7 + length]
        msg = data.split("@")[0]
        ack = None if "@" not in data else data.split("@")[1]
        ack = int(ack)
        distance = None

        return src_addr, msg, ack, distance

    def parse_U(self, packet: str):
        """Unicast ; packet format: #Unnddd"""

        msg_type = packet[:2]
        # verify that the message is a unicast message
        if msg_type != "#U":
            print("Received a broadcast message in unicast callback")
            return

        src_addr = None
        length = int(packet[5:7])

        data = packet[7 : 7 + length]
        msg = data.split("@")[0]
        ack = None if "@" not in data else data.split("@")[1]
        ack = int(ack)
        distance = None

        return src_addr, msg, ack, distance

    def parse_R(self, packet: str):
        """Acknoledgement ; packet format: #RxxxTyyyyy"""

        msg_type = packet[:2]
        # verify that the message is a unicast message
        if msg_type != "#R":
            print("Received a broadcast message in unicast callback")
            return

        src_addr = packet[2:5]
        msg = None
        ack = None
        distance = packet[7:12] * 1500 * 3.125e-5

        return msg, src_addr, ack, distance

    def parse_TO(self, packet: str):
        """Timeout ; packet format: #TO"""
        return None, None, None, None

    def on_receive_msg_logging(self, src_addr: str, msg: str, ack: int, distance: int):
        """Logs the message into a file"""

        with open("underwater_coms_recv.log", "a+") as f:
            if distance is not None:
                f.write(f"[{time.time()}][RECV][src:{src_addr}][dist:{distance}]\n")
            elif msg is None:
                f.write(f"[{time.time()}][RECV][src:{src_addr}][ACK:{ack}]\n")
            else:
                f.write(f"[{time.time()}][RECV][src:{src_addr}][ack:{ack}][dist:{distance}] {msg}\n")

    def on_receive_msg(self, src_addr: str, msg: str, ack: int, distance: int):
        """Callback for received message"""
        if msg is None:
            return

        print(f"[{time.time()}][RECV][src:{src_addr}][ack:{ack}][dist:{distance}] {msg}")

    def on_receive_flash_led(self, src_addr: str, msg: str, ack: int, distance: int):
        """flashes the LED when packet is received"""
        led.on_recv_msg()

    def on_receive_ack(self, src_addr: str, msg: str, ack: int, distance: int):
        """Callback for received ack"""
        if ack is None:
            return

        # acknoledgement of the reception of a message
        if msg is None:
            self.send_ack(ack)
            return

        # ack of a message previously sent
        # adding the ack to the list of received acks
        for packet in self.in_transit:
            if packet[3] == ack:
                self.ack_received.append(ack)
                return

        print(f'[WARNING] Received ack: "{ack}" but no corresponding message found, maybe timed out?')

    def on_send_msg_logging(self, dst_addr: str, msg: str, ack: int):
        """Logs the message into a file"""
        with open("underwater_coms_send.log", "a+") as f:
            if msg is None:
                f.write(f"[{time.time()}][SEND][dst:{dst_addr}][ACK:{ack}]\n")
            else:
                f.write(f"[{time.time()}][SEND][dst:{dst_addr}][ack:{ack}] {msg}\n")

    def on_receive_handshake(self, msg: str):
        if msg == "handshake":
            print("Handshake received")
            self.received_handshake = True

    def handshake_start(self):
        """waits for a handshake to continue"""

        self.received_handshake = False
        if self.on_receive_handshake not in self.recv_callbacks:
            self.recv_callbacks.append(self.on_receive_handshake)

        # sending handshake (30s timeout)
        msg = "handshake"
        self.send_msg(msg, ack=42, priority=0)
        print("Waiting for handshake...")
        while not self.received_handshake:
            if len(self.in_transit) == 0:
                break
            time.sleep(0.1)

        # check if timeout
        if self.received_handshake:
            print("Handshake successful")
        else:
            print("Handshake failed")

        return self.received_handshake

    def dummy_callback(self, src_addr: str, msg: str, ack: int, distance: int):
        pass

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


def manual_coms():
    modem = Modem()
    while True:
        msg = input("Enter message: ")
        modem.send_msg(msg, priority=1)

if __name__ == "__main__":
    modem = Modem()
    modem.send_msg("this is a test")
    modem.send_msg("blabla")
    modem.send_msg("omg this is insane !!!")

    # led.on_send_msg()
    # led.on_recv_msg()
