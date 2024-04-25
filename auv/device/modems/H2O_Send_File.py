"""
This is the client code to send a file to an H2O tablet. 

The payload capacity is 64 bytes:
    - First byte of the payload is the packet sequencing number
    - Sequence number 0 is a special packet that carries metadata
    - Metadata packets must be acknowledged by the receiver.
    - Metadata format is: 
        <PacketType(I|EOF)>,<comma-separated parameters>
    
    - Format for the Init packet:
        I, FILE_NAME, TOTAL_CHUNKS, ACK_INTERVAL
    - Format for the End Of File Packet:
        EOF, LAST_PACKET_SEQUENCE_NUMBER

    * EOF represents "End-Of-File", used as part of the metadata format to signal the end of transmission.

Notes on terminology:
    - Client code is an application/system that initiates contact with an outside device, known as a server. In a client system, the client sends requests to the server and receives responses.
    - 
    - Metadata is data that provides information about other data
"""

import collections
import signal
import sys
import threading
import time
from collections import namedtuple
from multiprocessing import Lock

import serial

# Set up the serial connection parameters
SERIAL_PORT = "/dev/h2observe"

ser = serial.Serial(port=SERIAL_PORT, baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

# Global variables for the serial connection
TxDelay = 1.3  # Assumed value for transmission delay of full payload packet
RTT = 3.0  # Round Trip Time assuming 2 Km range in water
max_seq_number = 0  # Sequence number of the last packet
last_send_packet = 0  # Last packet sequence number sent to the server
client_buffer = collections.OrderedDict()  # Will act as a buffer and keep all the packets that are to be sent to the receiver
gaps = []  # List of sequence numbers that are reported as missing and need to be retransmitted

# Will keep a track whether sending has completed or not
sending_completed = False

# Maintain the time to be used for report generation
t_start = 0
t_end = 0

# Filename is taken from the command line
FILE_NAME = "image.jpg"
ACK_INTERVAL = 50
PAYLOAD_SIZE = 63  # One byte is consumed by the sequence number, so 64 - 1 = 63 possible bytes
last_sent_seq = 0
first_ack_received = False
first_send_time = 0
first_ack_time = 0

def send_packet(packet, seq):
    """
    Send the packet to the host (over the serial connection)

    Args:
        packet: The packet to send
        seq: Last packet -- this is the sequence number and is not part of the actual message payload
    """
    global last_sent_seq, first_send_time

    print("sending:", packet)
    ser.write(packet) # Write the packet (send it) to through the serial connection
    print("Sent seq no: ", seq)
    last_sent_seq = seq
    if seq == 0:
        first_send_time = time.time() # Start the time log if this is the first message sent

def sequence_send(file_content):
    """
    Send all packets in the buffer in order (sequentially)

    Args:
        file_content: Unused, is the buffer that contains the messages to send
    """
    global last_send_packet, client_buffer, t_start, first_ack_received, TxDelay, RTT

    # Timer starts as first packet is sent from here
    t_start = time.time()
    print("client_buffer len :", len(client_buffer))

    # While there are still more messages to send
    while last_send_packet < len(client_buffer) - 1:
        # The first packet must be acknowledged, so do not increment the counter indicating the packet to send if the first ack is not received
        if first_ack_received:
            last_send_packet = last_send_packet + 1
        send_packet(client_buffer[last_send_packet], last_send_packet)
        print("sequence_send:", last_send_packet)

        # Wait to let transmission complete
        time.sleep(TxDelay)

        # If it's time to get an acknowledgment or if no more messages are left to be sent, then sleep to allow for the acknowledgment to be received,
        # then handle any gaps.
        if last_send_packet % ACK_INTERVAL == 0 or last_send_packet == len(client_buffer):
            # RTT sleep to allow for the acknowledgment to be received
            time.sleep(RTT)
            gaps_handler() # Handle the gaps

def receive_loop():
    """
    Creates an infinite loop to monitor the incoming acknowledgments and send the remaining packets (the gaps in the packet that need to be retransmitted)
    """
    print("started ack thread")
    while 1:
        rcv_bytes = ser.readline() # Read the received message
        ack_process(rcv_bytes) # Process the message


def ack_process(reply):
    """
    Processes the acknowledgment messages received, and transmits any gaps in the packet

    Args:
        reply: Received acknowledgment message
    """
    global client_buffer, sending_completed, t_end, t_start, t_total, gaps, first_ack_received, first_send_time, first_ack_time, TxDelay, RTT

    # Example ACK message: $R,001,BRD,ACK:10GAPS:4,5\r\n

    reply_str = reply.decode() # Turn the message from bytes to a string
    print("Reply:", reply_str)

    # Handle invalid start bytes
    if not (reply_str[0] == "$" and reply_str[1] == "R"):
        return

    # If the incoming packet is an acknowledgment (ACK) message
    if ",ACK:" in reply_str:
        s = reply_str.split("ACK:") # Split the string into two parts: everything before "ACK:" and everything after "ACK:". This removes "ACK:" from the message as well
        s1 = s[1].split("GAPS:") # Split the string after "ACK:" further into the stuff before "GAPS:", which is the sequence number, and the stuff after "GAPS:", which is the number of gaps (plus other stuff)

        # Retrieve the sequence number in the ACK to get the last acknowledged packet
        current_ack_seq_number = int(s1[0])
        print("ack seq no : ", current_ack_seq_number)

        # If this is the first acknowledged packet
        if current_ack_seq_number == 0:
            first_ack_received = True
            first_ack_time = time.time()
            round_trip = first_ack_time - first_send_time # Compute the round trip time
            print("First ack round trip time:", round_trip)
            RTT = round_trip + 0.3 # Set the round trip time from now on based on the calculated time

        gaps = []

        # Split the string so we get the number of gaps in the transmitted packet
        if len(s1) > 1:
            s2 = s1[1].split("\r\n")
            gaps = s2[0].split(",")
        print("ack seq no : ", current_ack_seq_number)
        print("gap seq nos : ", gaps)

        # If the current acknowlegment sequence number is equal to the sequence number of the last packet, handle any gaps in the message then exit
        if current_ack_seq_number == max_seq_number:
            print("ack_eof")
            last_ack_gaps = len(gaps) # Find the number of gaps
            gaps_handler() # Handle gaps

            # If there are no more gaps, then exit
            if last_ack_gaps == 0:
                t_end = time.time()
                t_total = t_end - t_start
                print("Total time:", t_total)
                sending_completed = True
                time.sleep(1)
                sys.exit("done")
            else:
                send_packet(client_buffer[max_seq_number], max_seq_number)
                # Timer for repeating EoF (End of File)
                signal.alarm(0) # Stop any previously running alarms/signals
                signal.setitimer(signal.ITIMER_REAL, TxDelay + RTT)

def timeout_thread(timeout_th, frame):
    """
    Handle the timeouts. This function is called with a signal.

    Args:
        timeout_th: Signal number
        frame: Current stack frame

    NOTE: These arguments are unused because they are not needed. These are placeholder arguments so that the function can conform to the signal.
    """
    global TxDelay, RTT
    # Send the EoF (End of File) packet
    send_packet(client_buffer[max_seq_number], max_seq_number)
    # Start the EoF timer again
    signal.alarm(0)
    signal.setitimer(signal.ITIMER_REAL, TxDelay + RTT)

def gaps_handler():
    """
    To handle retransmitting any gaps that may have been present in the message
    """
    global gaps, TxDelay, last_send_packet

    print("gaps_handler", gaps)

    # For each gap in the list, check whether the sequence number of the gap is less than or equal to the last sent packet.
    # If this is the case, that means there is a valid gap, so we retransmit the corresponding packet, and sleep for the designated amount of time to 
    # allow for complete transmission.
    for g in gaps:
        if int(g) <= last_send_packet:
            print(f"Sending gap sequence number = {str(int(g))}")
            send_packet(client_buffer[int(g)], int(g))
            time.sleep(TxDelay)
    gaps = []

def main():
    """
    Main function for running the code to send files/packets to the H2O tablet
    """
    """
    NOTE: Big-endian byte format order orders the message from most significant bytes (the bytes that have the most numerical significance/the biggest) to least significant byte

    For example, in the number 11001010, the leftmost byte, "1", would be the most significant.
    """
    global client_buffer, max_seq_number, ACK_INTERVAL, PAYLOAD_SIZE

    if len(sys.argv) < 2:
        print("Usage: ", sys.argv[0], "file_to_send <ack_interval>")
        sys.exit("")

    FILE_NAME = sys.argv[1]

    if len(sys.argv) > 2:
        ACK_INTERVAL = int(sys.argv[2])

    print("File:", FILE_NAME, " ACK_INTERVAL", ACK_INTERVAL)

    # Read the data from the file to be sent over and break it into chunks based on PAYLOAD_SIZE
    # Store the chunks in client_buffer, the buffer to store the packets, by its sequence number, so that the packets are sent in the correct order
    sequence_number = 0
    max_seq_number = 0

    try:
        # Find the total number of chunks necessary
        chunk_count = 0
        with open(FILE_NAME, "rb") as f:
            while True:
                chunk = f.read(int(PAYLOAD_SIZE))
                if chunk:
                    chunk_count = chunk_count + 1
                else:
                    break

        # Add an initialization chunk to the beginning of the buffer (file)
        int_seq = 0
        seq = int_seq.to_bytes(1, "big") # Convert the initialization sequence number to bytes (length 1, byte format "big-endian" order)
        prefix = "$S,TOP,BRD," # Will be the packet header
        suffix = f'I,{FILE_NAME},{str(chunk_count)},{str(ACK_INTERVAL)}\r\n' # Will be metadata (file name, number of chunks in the file/buffer, acknowledgment interval)
        client_buffer[sequence_number] = prefix.encode() + seq + suffix.encode() # Put the initialization packet in the buffer at the beginning
        print(client_buffer[sequence_number].decode())
        sequence_number = sequence_number + 1 # Shift all of the other packets one unit to the right to accomodate the new init chunk

        # Open the file name specified in binary read mode
        with open(FILE_NAME, "rb") as f:
            while True:
                # For each non-empty chunk in the file, create a packet
                # Literally the only difference between a chunk and a packet is that you put a prefix and a sequence number before the chunk (the message payload), and then put 
                # the suffix ("\r\n") at the end. Then you encode that packet into byte form.
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

                # Really there are 256 possible sequence numbers, but you start at 0, which will be the initialization chunk, and then the last chunk, at spot 255, will be the EoF chunk
                if sequence_number > 254:
                    break

        # Add the EoF (End of File) chunk
        max_seq_number = sequence_number
        int_seq = 0
        seq = int_seq.to_bytes(1, "big")
        prefix = "$S,TOP,BRD,"
        suffix = f'EOF,{str(sequence_number)},\r\n'
        client_buffer[sequence_number] = prefix.encode() + seq + suffix.encode()
        sequence_number = sequence_number + 1
    except Exception as e:
        print(e)
        sys.exit(f"Failed to open file: {FILE_NAME}")

    # Initialize the signal thread that will help in timeout tracking.
    signal.signal(signal.SIGALRM, timeout_thread)

    # Initialize the ack thread that will monitor the incoming acknowledgment packets.
    ack_thread = threading.Thread(target=receive_loop)
    ack_thread.start()

    # Do initial packet sending for all sequence numbers. This means send all of the packets in the order of how they are stored in the buffer, but don't check for gaps. This will be
    # done by the ack_thread.
    sequence_send(client_buffer)

    # Monitor if the sending is complete (infinitely until the sending has been completed)
    while 1:
        if sending_completed:
            break

    # Continue receving acknowledgment messages and retransmitting the packets to fill the gaps, until the file has been sent fully
    ack_thread.join()

    # Close the serial connection
    ser.close()


if __name__ == "__main__":
    main()
