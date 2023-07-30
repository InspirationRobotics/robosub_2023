import os
import time
import serial
from ..utils.deviceHelper import dataFromConfig

# torpedo channel 2:
# load: 2400
# Torpedo 1: 1700 (1800 actual)
# Torpedo 2: 1300 (1400 actual
#
# marker dropper channel 1:
# Load: 1600
# Ball 1: 1200
# Ball 2: 700
#
# gripper channel 0:
# no incremental control; just open and close
# open: 1550 1.5 seconds then 1500 to stop
# close: 1450 1.5 seconds then 1500 to stop


class Polulu:
    def __init__(self):
        self.USB = serial.Serial(port=dataFromConfig("polulu"))

        self.torpedo_state = {0: (2, 2400), 1: (2, 1700), 2: (2, 1300)}
        self.dropper_state = {0: (1, 1600), 1: (1, 1200), 2: (1, 700)}
        self.gripper_state = {0: (0, 1500), 1: (0, 1550), 2: (0, 1450)}

        if not self.USB.isOpen():
            self.USB.open()

            # set to default state
            self.set_pwm(*self.torpedo_state[0])
            self.set_pwm(*self.dropper_state[0])
            self.set_pwm(*self.gripper_state[0])
        else:
            print("[INFO] Polulu serial is already open")

    def set_pwm(self, channel, target):
        target = target * 4
        lsb = target & 0x7F
        msb = (target >> 7) & 0x7F
        cmd = chr(0x84) + chr(channel) + chr(lsb) + chr(msb)
        self.USB.write(bytes(cmd.encode()))


class Torpedo(Polulu):
    def __init__(self):
        super().__init__()

        # state 3 = all torpedos fired, need to reload
        self.state = 0

    def fire(self, torpedo_num=-1):
        if torpedo_num == -1:
            self.state += 1
            torpedo_num = self.state

        if torpedo_num == 3:
            print("[INFO] All torpedos fired, need to reload")
            return

        elif torpedo_num not in self.torpedo_state.keys():
            print(f"[ERROR] Invalid torpedo number: {torpedo_num}")
            return

        # fire torpedo
        self.set_pwm(*self.torpedo_state[torpedo_num])
        print(f"[INFO] Fired torpedo #{torpedo_num}")

    def reload(self):
        # manual action to reload the torpedos
        reloaded = False
        while not reloaded:
            inp = input("Reloaded? (y/n): ")
            if inp == "y":
                reloaded = True

        self.state = 0


class Dropper(Polulu):
    def __init__(self):
        super().__init__()

        # state 3 = all balls dropped, need to reload
        self.state = 0

    def drop(self, ball_num=-1):
        if ball_num == -1:
            self.state += 1
            ball_num = self.state

        if ball_num == 3:
            print("[INFO] All balls dropped, need to reload")
            return

        elif ball_num not in self.dropper_state.keys():
            print(f"[ERROR] Invalid ball number: {ball_num}")
            return

        # drop ball
        self.set_pwm(*self.dropper_state[ball_num])
        print(f"[INFO] Dropped ball #{ball_num}")

    def reload(self):
        # manual action to reload the dropper
        reloaded = False
        while not reloaded:
            inp = input("Reloaded? (y/n): ")
            if inp == "y":
                reloaded = True

        self.state = 0


class Gripper(Polulu):
    """
    Gripper class for the gripper servo

    /!\ Please test this code for gripper, the 0.5 seconds will likely need to be adjusted
    If gripper keeps pushing after reaching its limit, it WILL KILL our power distribution board
    """

    def __init__(self):
        super().__init__()

        # state 0 = stop (default)
        # state 1 = open
        # state 2 = close
        self.state = 0

    def open(self):
        # ensure gripper is not already open
        if self.state == 1:
            return

        # open gripper
        start_time = time.time()
        while time.time() - start_time < 0.5:
            self.set_pwm(*self.gripper_state[1])
            time.sleep(0.05)

        # stop gripper
        self.set_pwm(*self.gripper_state[0])
        self.set_pwm(*self.gripper_state[0])
        self.state = 1

    def close(self):
        # ensure gripper is not already closed
        if self.state == 2:
            return

        # close gripper
        start_time = time.time()
        while time.time() - start_time < 0.5:
            self.set_pwm(*self.gripper_state[2])
            time.sleep(0.05)

        # stop gripper
        self.set_pwm(*self.gripper_state[0])
        self.set_pwm(*self.gripper_state[0])
        self.state = 2


class Servo:
    def __init__(self):
        self.USB = serial.Serial(port=dataFromConfig("polulu"))  # need to find id
        self.USB.isOpen()
        self.gripState = True
        self.torpedoState = 0
        self.ballState = 0
        self.torpedo(0)
        self.dropper(0)
        print("Initialized...")

    def setPwm(self, channel, target):
        target = target * 4
        lsb = target & 0x7F  # 7 bits for least significant byte
        msb = (target >> 7) & 0x7F  # shift 7 and take next 7 bits for msb
        cmd = chr(0x84) + chr(channel) + chr(lsb) + chr(msb)
        self.USB.write(bytes(cmd.encode()))

    # please test this code for gripper, the 1.5 seconds will likely need to be adjusted (should be close though)
    # if gripper keeps pushing after reaching its limit, it WILL KILL our power distribution board
    def gripper(self, state):  # true for to open, false for to close
        if self.gripState == state:
            return
        pwm = 1500
        if state:
            pwm = 1550
        else:
            pwm = 1450
        startTime = time.time()
        while time.time() - startTime < 1.5:  # run for 1.5 seconds
            self.setPwm(0, pwm)
            time.sleep(0.05)
        self.setPwm(0, 1500)  # stops gripper
        self.setPwm(0, 1500)  # redundant
        self.gripState = state

    def torpedo(self, torpedoNum=-1):  # 0,1,2 : load, fire 1, fire 2
        if torpedoNum == 0:  # load state
            self.setPwm(2, 2400)
            self.torpedoState = 0
            print("Torpedo in Load State")
        elif torpedoNum == 1:  # fire torpedo 1
            self.setPwm(2, 1700)
            print("Torpedo 1 fired")
        elif torpedoNum == 2:  # fire torpedo 2
            self.torpedo(1)  # incase first has not been fired yet
            time.sleep(0.5)
            self.setPwm(2, 1300)
            print("Torpedo 2 fired")
            time.sleep(1)
            self.torpedo(0)  # reset to load position now that both are fired
        elif torpedoNum == -1:
            self.torpedoState += 1
            self.torpedo(self.torpedoState)
        else:
            print("Invalid Arg in [torpedoLauncher]")

    def dropper(self, ball=-1):  # 0,1,2 : load, drop 1, drop 2
        if ball == 0:  # load balls
            self.setPwm(1, 1600)
            self.ballState = 0
        elif ball == 1:  # Drop first ball
            self.setPwm(1, 1200)
        elif ball == 2:  # Drop second ball
            self.dropper(1)  # incase dropper(1) is not called before allows drop of both balls without collision
            time.sleep(0.5)
            self.setPwm(1, 700)
            time.sleep(1)
            self.dropper(0)  # resets to load position now that its empty
        elif ball == -1:
            self.ballState += 1
            self.dropper(self.ballState)
        else:
            print("Invalid Arg in [dropper]")
