# This is the client code to send file to H2O tablet
# payload capacity is 64 bytes
# first byte of the payload is used for packet sequency number
# sequence number 0 is a special packet that carries metadata
# Metadata packets must be acknowledged by receiver
# Metadat format is : <PaketType(I|EOF)>,<comma separated parameters>
#  For Init Packet : I,FILE_NAME,TOTAL_CHUNKS,ACK_INTERVAL
#  For End Of File Packet : EOF,LAST_PACKET_SEQUENCE_NUMBER


import collections
import signal
import sys
import threading
import time
from collections import namedtuple
from multiprocessing import Lock

import serial


SERIAL_PORT = "/dev/h2observe"

ser = serial.Serial(port=SERIAL_PORT, baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

# Globals
TxDelay = 1.3  # Assumed value for tranmission delay of full payload packet
RTT = 3.0  # Round Trip Time assuming 2km range in water
max_seq_number = 0  # Sequence number of the last packet
last_send_packet = 0  # Last packet sequence number sent to the server
client_buffer = collections.OrderedDict()  # Will act as a buffer and keep all the packets that are to be sent to the receiver
gaps = []  # list of sequence number that are reported as missing and need to be retransmitted

# Will keep a track whether sending has completed or not
sending_completed = False

# Maintain the time to be used for report generation
t_start = 0
t_end = 0

# Filename is taken from the command line
FILE_NAME = "image.jpg"
ACK_INTERVAL = 50
PAYLOAD_SIZE = 63  # one byte is consumed by seqno
last_sent_seq = 0
first_ack_received = False
first_send_time = 0
first_ack_time = 0


# Send the packet to the host details in the parameters
def send_packet(packet, seq):
    global last_sent_seq, first_send_time

    print("sending:", packet)
    ser.write(packet)
    print("Sent seq no: ", seq)
    last_sent_seq = seq
    if seq == 0:
        first_send_time = time.time()


# Send all packets in the buffer sequentially
def sequence_send(file_content):
    global last_send_packet, client_buffer, t_start, first_ack_received, TxDelay, RTT
    # Timer starts as first packet is send from here
    t_start = time.time()
    print("client_buffer len :", len(client_buffer))
    while last_send_packet < len(client_buffer) - 1:
        # first packet must be acked, so do not increment if first ack is not received
        if first_ack_received:
            last_send_packet = last_send_packet + 1
        send_packet(client_buffer[last_send_packet], last_send_packet)
        print("sequence_send:", last_send_packet)
        # wait to let transmission complete
        time.sleep(TxDelay)
        if last_send_packet % ACK_INTERVAL == 0 or last_send_packet == len(client_buffer):
            # RTT sleep to allow for ack to be received
            time.sleep(RTT)
            gaps_handler()


# read loop to monitor the incoming ack's and sent the remaining packets
def receive_loop():
    print("started ack thread")
    while 1:
        rcv_bytes = ser.readline()
        ack_process(rcv_bytes)


def ack_process(reply):
    global client_buffer, sending_completed, t_end, t_start, t_total, gaps, first_ack_received, first_send_time, first_ack_time, TxDelay, RTT

    # Receive the ACK e.g
    # $R,001,BRD,ACK:10GAPS:4,5\r\n

    reply_str = reply.decode()
    print("Reply:", reply_str)
    if not (reply_str[0] == "$" and reply_str[1] == "R"):
        # invalid start bytes
        return

    # If incoming packet is an ACK
    if ",ACK:" in reply_str:
        s = reply_str.split("ACK:")
        s1 = s[1].split("GAPS:")
        # Retrieve the sequence number in the ACK to get the last ACK'ed packet
        current_ack_seq_number = int(s1[0])
        print("ack seq no : ", current_ack_seq_number)
        if current_ack_seq_number == 0:
            first_ack_received = True
            first_ack_time = time.time()
            round_trip = first_ack_time - first_send_time
            print("First ack round trip time:", round_trip)
            RTT = round_trip + 0.3
        gaps = []
        if len(s1) > 1:
            s2 = s1[1].split("\r\n")
            gaps = s2[0].split(",")
        print("ack seq no : ", current_ack_seq_number)
        print("gap seq nos : ", gaps)
        if current_ack_seq_number == max_seq_number:
            print("ack_eof")
            last_ack_gaps = len(gaps)
            gaps_handler()
            if last_ack_gaps == 0:
                t_end = time.time()
                t_total = t_end - t_start
                print("Total time:", t_total)
                sending_completed = True
                time.sleep(1)
                sys.exit("done")
            else:
                send_packet(client_buffer[max_seq_number], max_seq_number)
                # timer for repeating EoF
                signal.alarm(0)
                signal.setitimer(signal.ITIMER_REAL, TxDelay + RTT)


# Handle the timeouts
def timeout_thread(timeout_th, frame):
    global TxDelay, RTT
    # send EoF
    send_packet(client_buffer[max_seq_number], max_seq_number)
    # start timer again
    signal.alarm(0)
    signal.setitimer(signal.ITIMER_REAL, TxDelay + RTT)


def gaps_handler():
    global gaps, TxDelay, last_send_packet

    print("gaps_handler", gaps)
    for g in gaps:
        if int(g) <= last_send_packet:
            print(f"Sending gap sequence number = {str(int(g))}")
            send_packet(client_buffer[int(g)], int(g))
            time.sleep(TxDelay)
    gaps = []


# Main function
def main():
    global client_buffer, max_seq_number, ACK_INTERVAL, PAYLOAD_SIZE

    if len(sys.argv) < 2:
        print("Usage: ", sys.argv[0], "file_to_send <ack_interval>")
        sys.exit("")

    FILE_NAME = sys.argv[1]

    if len(sys.argv) > 2:
        ACK_INTERVAL = int(sys.argv[2])

    print("File:", FILE_NAME, " ACK_INTERVAL", ACK_INTERVAL)

    # Read the data from the file to be sent over and break it into chunks based on PAYLOAD_SIZE
    # Store the chunkks in client_buffer based on the sequence number
    sequence_number = 0
    max_seq_number = 0
    try:
        # find total chunk count
        chunk_count = 0
        with open(FILE_NAME, "rb") as f:
            while True:
                chunk = f.read(int(PAYLOAD_SIZE))
                if chunk:
                    chunk_count = chunk_count + 1
                else:
                    break
        # add Init chunk
        int_seq = 0
        seq = int_seq.to_bytes(1, "big")
        prefix = "$S,TOP,BRD,"
        suffix = f'I,{FILE_NAME},{str(chunk_count)},{str(ACK_INTERVAL)}\r\n'
        client_buffer[sequence_number] = prefix.encode() + seq + suffix.encode()
        print(client_buffer[sequence_number].decode())
        sequence_number = sequence_number + 1

        with open(FILE_NAME, "rb") as f:
            while True:
                chunk = f.read(int(PAYLOAD_SIZE))
                if chunk:
                    max_seq_number = sequence_number
                    # chunk_checksum=compute_checksum_for_chuck(sequence_number,chunk)
                    seq = sequence_number
                    int_seq = seq
                    seq = int_seq.to_bytes(1, "big")
                    prefix = "$S,TOP,BRD,"
                    client_buffer[sequence_number] = prefix.encode() + seq + chunk + "\r\n".encode()
                    # print(client_buffer)
                    sequence_number = sequence_number + 1
                else:
                    break
                if sequence_number > 254:
                    break
        # add EoF chunk
        max_seq_number = sequence_number
        int_seq = 0
        seq = int_seq.to_bytes(1, "big")  # type data
        prefix = "$S,TOP,BRD,"
        suffix = f'EOF,{str(sequence_number)},\r\n'
        client_buffer[sequence_number] = prefix.encode() + seq + suffix.encode()
        sequence_number = sequence_number + 1
    except Exception as e:
        print(e)
        sys.exit(f"Failed to open file: {FILE_NAME}")

    # Initialize the signal thread that will help in timeout tracking
    signal.signal(signal.SIGALRM, timeout_thread)
    # Initialize the ack thread that will monitor the incoming ack's and sent the remaining packets
    ack_thread = threading.Thread(target=receive_loop)
    ack_thread.start()
    # Do initial packet sending for all sequence numbers
    sequence_send(client_buffer)
    # Monitor if the sending is complete
    while 1:
        if sending_completed:
            break
    # blocks the main thread until the ack thread is terminated
    ack_thread.join()
    ser.close()


if __name__ == "__main__":
    main()
