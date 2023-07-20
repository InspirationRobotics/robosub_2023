import signal
import time
import math

import serial
import threading


from ...utils.deviceHelper import dataFromConfig


class DVL:
    """DVL class to enable position estimation"""

    def __init__(self, autostart=True):
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
        self.newData = False
        self.prev_time = None

        # sensor error
        self.heading_error = 1.0  # deg/s
        self.dvl_error = 0.001  # m/s

        # [0 -> 1], close to 0 is perfect, 1 is random basically
        self.error_rate = ((self.heading_error / 360) + self.dvl_error)
        self.accumulated_error = 0  # accumulated error (+/- m)
        
        self.compass = None
        self.vel_rot = [0, 0, 0]  # rotated velocity vector
        self.position = [0, 0, 0] # position in meters
        self.is_valid = False

        if autostart:
            self.start()

    def __parseLine(self, line):
        """Parse line"""
        return line.decode("utf-8").replace(" ", "").replace("\r\n", "").split(",")

    def __get_velocity(self):
        """Get velocity"""
        data = {
            "Timestamp": [],  # year, month, day, hour:minute:second
            "Attitude": [],  # roll, pitch, and heading in degrees
            "Salinity": [],  # in ppt (parts per thousand)
            "Temp": [],  # celcius
            "Transducer_depth": [],  # meters
            "Speed_of_sound": [],  # meters per second
            "Result_code": [],
            "DVL_velocity": [],  # mm/s # xyz error
            "isDVL_velocity_valid": [],  # boolean
            "AUV_velocity": [],  # mm/s # xyz
            "isAUV_velocity_valid": [],  # boolean
            "Distance_from_bottom": [],  # meters
            "Time_since_valid": [],  # seconds
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

    def __process_data(self, packet):
        """integrate velocity into position"""

        vel = dataPacket.get("AUV_velocity", [0, 0, 0])  # mm/s # xyz
        current_time = dataPacket.get("Time", 0)  # seconds

        if self.prev_time is None or self.compass is None:
            self.prev_time = current_time
            print("[WARN] DVL not ready, waiting for compass or some more sample")
            return False

        dt = current_time - self.prev_time
        if self.dt < 0:
            print("[WARN] DVL time error, skipping")
            return False

        self.is_valid = data["isAUV_velocity_valid"]
        if not self.is_valid:
            print("[WARN] DVL velocity not valid, skipping")
            return False

        self.prev_time = current_time

        # rotate velocity vector using compass heading
        # Y = forward, X = lateral, Z = vertical
        vel_rot = [
            vel[0] * math.cos(self.compass) - vel[1] * math.sin(self.compass),
            vel[0] * math.sin(self.compass) + vel[1] * math.cos(self.compass),
            vel[2],
        ]

        # in metters per second
        self.vel_rot = [
            vel_rot[0] / 1000,
            vel_rot[1] / 1000,
            vel_rot[2] / 1000,
        ]

        # integrate velocity to position with respect to time
        self.position = [
            self.position[0] + self.vel_rot[0] * dt,
            self.position[1] + self.vel_rot[1] * dt,
            self.position[2] + self.vel_rot[2] * dt,
        ]

        self.accumulated_error += math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2) * dt * self.error_rate
        return True

    def reset_position(self):
        """Reset position to 0"""
        self.position = [0, 0, 0]

    def update(self):
        """Update DVL data (runs in a thread)"""
        while self.__running:
            while self.ser.in_waiting and self.__running:
                vel_packet = self.__get_velocity(self.ser.readline())
                if vel_packet is None:
                    continue

                ret = self.__process_data(vel_packet)
                self.newData = ret

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
