import signal
import time
import math

import serial
import threading


from ...utils.deviceHelper import dataFromConfig


class DVL:
    """DVL class to enable position estimation"""

    def __init__(self, autostart=True, test=False):
        self.test = test
        if not self.test:
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

        self.__running = False
        self.__thread_vel = None
        self.prev_time = None

        # sensor error
        self.compass_error = math.radians(1.0)  # rad/s
        self.dvl_error = 0.001  # m/s
        self.error = [0, 0, 0]  # accumulated error

        # NORTH = 0, EAST = pi/2, SOUTH = pi, WEST = 3pi/2
        self.compass_rad = None  # rad

        self.vel_rot = [0, 0, 0]  # rotated velocity vector
        self.position = [0, 0, 0]  # position in meters
        self.is_valid = False
        self.data_available = False

        # stores position and error history for context manager
        self.position_memory = []
        self.error_memory = []

        if autostart:
            self.start()

    def __parseLine(self, line):
        """Parse line"""
        return line.decode("utf-8").replace(" ", "").replace("\r\n", "").split(",")

    def __get_velocity(self):
        """Get velocity"""
        data = {
            "Time": 0,  # year, month, day, hour:minute:second
            "Attitude": [],  # roll, pitch, and heading in degrees
            "Salinity": 0,  # in ppt (parts per thousand)
            "Temp": 0,  # celcius
            "Transducer_depth": 0,  # meters
            "Speed_of_sound": [],  # meters per second
            "Result_code": 0,
            "DVL_velocity": [],  # mm/s # xyz
            "isDVL_velocity_valid": False,  # boolean
            "AUV_velocity": [],  # mm/s # xyz
            "isAUV_velocity_valid": False,  # boolean
            "Distance_from_bottom": 0,  # meters
            "Time_since_valid": 0,  # seconds
        }
        SA = self.__parseLine(self.ser.readline())
        if SA[0] != ":SA":
            return None

        TS = self.__parseLine(self.ser.readline())
        BI = self.__parseLine(self.ser.readline())
        BS = self.__parseLine(self.ser.readline())
        BE = self.__parseLine(self.ser.readline())  # unused
        BD = self.__parseLine(self.ser.readline())

        milli = int(TS[1][12:14]) * 0.01
        seconds = int(TS[1][10:12])
        minutes = int(TS[1][8:10]) * 60
        hours = int(TS[1][6:8]) * 60 * 60
        time = hours + minutes + seconds + milli

        data["Attitude"] = [float(SA[1]), float(SA[2]), float(SA[3])]
        data["Time"] = time
        data["Salinity"] = float(TS[2])
        data["Temp"] = float(TS[3])
        data["Transducer_depth"] = float(TS[4])
        data["Speed_of_sound"] = float(TS[5])
        data["Result_code"] = TS[6]
        data["DVL_velocity"] = [int(BI[1]), int(BI[2]), int(BI[3]), int(BI[4])]
        data["isDVL_velocity_valid"] = BI[5] == "A"
        data["AUV_velocity"] = [int(BS[1]), int(BS[2]), int(BS[3])]
        data["isAUV_velocity_valid"] = BS[4] == "A"
        data["Distance_from_bottom"] = float(BD[4])
        data["Time_since_valid"] = float(BD[5])

        return data

    def process_packet(self, packet):
        """integrate velocity into position"""

        vel = packet.get("AUV_velocity", [0, 0, 0])  # mm/s # xyz
        current_time = packet.get("Time", 0)  # seconds

        if self.prev_time is None or self.compass_rad is None:
            self.prev_time = current_time
            print("[WARN] DVL not ready, waiting for compass or some more sample")
            return False

        dt = current_time - self.prev_time
        if dt < 0:
            print("[WARN] DVL time error, skipping")
            return False

        self.is_valid = packet["isAUV_velocity_valid"]
        if not self.is_valid:
            print("[WARN] DVL velocity not valid, skipping")
            return False

        self.prev_time = current_time
        # convert vel to m/s
        vel = [
            vel[0] / 1000,
            vel[1] / 1000,
            vel[2] / 1000,
        ]

        # rotate velocity vector using compass heading
        # X = lateral, Y = forward, Z = vertical
        self.vel_rot = [
            vel[0] * math.cos(self.compass_rad) + vel[1] * math.sin(self.compass_rad),
            vel[1] * math.cos(self.compass_rad) - vel[0] * math.sin(self.compass_rad),
            vel[2],
        ]

        # integrate velocity to position with respect to time
        self.position = [
            self.position[0] + self.vel_rot[0] * dt,
            self.position[1] + self.vel_rot[1] * dt,
            self.position[2] + self.vel_rot[2] * dt,
        ]

        vel_rot_error = [
            (vel[0] + self.dvl_error) * math.cos(self.compass_rad + self.compass_error)
            + (vel[1] + self.dvl_error) * math.sin(self.compass_rad + self.compass_error),
            (vel[1] + self.dvl_error) * math.cos(self.compass_rad + self.compass_error)
            - (vel[0] + self.dvl_error) * math.sin(self.compass_rad + self.compass_error),
            vel[2] + self.dvl_error,  # we actually have a sensor for depth, so useless
        ]

        # calculate accumulated error
        self.error = [
            self.error[0] + abs(self.vel_rot[0] - vel_rot_error[0]) * dt,
            self.error[1] + abs(self.vel_rot[1] - vel_rot_error[1]) * dt,
            self.error[2] + abs(self.vel_rot[2] - vel_rot_error[2]) * dt,
        ]

        return True

    def reset_position(self):
        """Reset position to 0"""
        self.position = [0, 0, 0]
        self.error = [0, 0, 0]

    def update(self):
        """Update DVL data (runs in a thread)"""
        while self.__running:
            while self.ser.in_waiting and self.__running:
                vel_packet = self.__get_velocity(self.ser.readline())
                if vel_packet is None:
                    continue
                ret = self.process_packet(vel_packet)
                self.data_available = ret

    def start(self):
        # ensure not running
        if self.__running or self.__thread_vel.is_alive():
            print("[WARN] DVL already running")
            return

        self.__running = True
        self.__thread_vel = threading.Thread(target=self.update)
        self.__thread_vel.start()

    def stop(self):
        self.__running = False
        self.__thread_vel.join()

    def __enter__(self):
        """Context manager for DVL"""
        if not self.__running and not self.test:
            self.start()

        # begin a context, store current position
        self.position_memory.append(self.position)
        self.error_memory.append(self.error)

        # reset position
        self.reset_position()

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Exit context manager"""
        prev_pos = self.position_memory.pop()
        prev_error = self.error_memory.pop()

        # restore previous position and error
        self.position = [
            self.position[0] + prev_pos[0],
            self.position[1] + prev_pos[1],
            self.position[2] + prev_pos[2],
        ]

        self.error = [
            self.error[0] + prev_error[0],
            self.error[1] + prev_error[1],
            self.error[2] + prev_error[2],
        ]