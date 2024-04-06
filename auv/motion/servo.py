"""
Handles sending PWM values to the Polulu servo controller for the dropper, gripper, and torpedoes.
Since a serial connection is being used, this file also handles encoding the PWM values into the proper format for the Polulu controller.
"""

import os 
import time
import serial # For serial communication (1 bit at a time)
from ..utils.deviceHelper import dataFromConfig # For getting the configuration from each device

"""
Gripper is channel 0:
    There is no incremental control, it just opens and closes
    
    PWMs:
        open: 1550 for 1.5 secs then 1500 to stop
        close: 1450 for 1.5 secs then 1500 to stop

Dropper is channel 1:
    PWMs:
        load: 1600
        ball 1: 1200
        ball 2: 700

Torpedo is channel 2:
    PWMs:
        load: 2400
        torpedo 1: 1700 (1800 actual)
        torpedo 2: 1300 (1400 actual)
"""

class Polulu:
    """
    Class for controlling the Polulu servo controller

    Attributes:
        USB: serial conection object for Polulu
        torpedo_state: Dictionary that maps channel numbers to the servo states for torpedoes
        dropper_state: Dictionary that maps channel numbers to the servo states for the dropper
        gripper_state: Dictionary that maps channel numbers to the servo states for the gripper
    """
    def __init__(self):
        """
        Initializes Polulu class and establishes serial connection to Polulu controller
        """
        self.USB = serial.Serial(port=dataFromConfig("polulu"))

        # Each tuple represents : (channel number, PWM value)
        # Set the initial states for the servos
        self.torpedo_state = {0: (2, 2400), 1: (2, 1700), 2: (2, 1300)}
        self.dropper_state = {0: (1, 1600), 1: (1, 1200), 2: (1, 700)}
        self.gripper_state = {0: (0, 1500), 1: (0, 1550), 2: (0, 1450)}

        # Open the USB serial connection
        if not self.USB.isOpen():
            self.USB.open()

            # Set PWMs to the default state
            self.set_pwm(*self.torpedo_state[0])
            self.set_pwm(*self.dropper_state[0])
            self.set_pwm(*self.gripper_state[0])
        else:
            print("[INFO] Polulu serial is already open")

    def set_pwm(self, channel, target):
        """
        Sets PWM value to control the servo
        Reconstructs the target PWM value, given in two bytes, into two different 7-bit bytes (lsb and msb)
        Args:
            channel (int): Channel number
            target (int): Target PWM value
        """
        target = target * 4
        lsb = target & 0x7F # Extracts the lower 7 bits of the target position to obtain the least significant byte
        msb = (target >> 7) & 0x7F # Calculates the most significant byte by shifiting the target position 7 bits to the right (extracts upper 7 bits after target shift)
        
        # Creates a command string to send to the Polulu servo controller
        # chr(0x84) : command byte indicating a set PWM command, chr(channel) : channel number of the servo motor
        # chr(lsb): least significant byte of target PWM value chr(msb): most significant byte of the PWM value
        cmd = chr(0x84) + chr(channel) + chr(lsb) + chr(msb)

        # Encode the command string and write it to the serial connection
        self.USB.write(bytes(cmd.encode()))


class Torpedo(Polulu):
    """
    Torpedo class for controlling torpedoes

    Inherits:
        Polulu: Parent class for servo control
        
    Attributes:
        state (int): State of the torpedoes
    """

    def __init__(self):
        """Initializes the torpedo object"""
        super().__init__() # Intitializes superclass

        # State == 1: Fire the first torpedo
        # State == 2: Fire the second torpedo
        # State == 3: Need to reload
        self.state = 0

    def fire(self, torpedo_num=-1):
        """
        Fires a torpedo

        Args:
            torpedo_num (int): Number of the torpedo to fire -- if -1 fires next avaliable torpedo
        """
        if torpedo_num == -1:
            self.state += 1
            torpedo_num = self.state

        if torpedo_num == 3:
            print("[INFO] All torpedos fired, need to reload")
            return

        elif torpedo_num not in self.torpedo_state.keys():
            print(f"[ERROR] Invalid torpedo number: {torpedo_num}")
            return

        # Set PWM value to fire the torpedo
        self.set_pwm(*self.torpedo_state[torpedo_num])
        print(f"[INFO] Fired torpedo #{torpedo_num}")

    def reload(self):
        """
        Sets reloaded to True after the user has manually reloaded the torpedoes and inputted 'y' into the computer
        """
        reloaded = False
        while not reloaded:
            inp = input("Reloaded? (y/n): ")
            if inp == "y":
                reloaded = True

        self.state = 0


class Dropper(Polulu):
    """
    Class to handle the Dropper

    Inherits:
        Polulu: Polulu class for servo control

    Attributes:
        state (int): The state of the dropper
    """
    def __init__(self):
        """
        Initializes Dropper object and sets the state of the dropper
        """
        super().__init__()

        # State 1: drop the first ball
        # State 2: drop the second ball
        # State 3: reload
        self.state = 0

    def drop(self, ball_num=-1):
        """
        To drop the ball

        Args:
            ball_num (int): The number of the ball to be dropped
        """
        if ball_num == -1:
            self.state += 1
            ball_num = self.state

        if ball_num == 3:
            print("[INFO] All balls dropped, need to reload")
            return

        elif ball_num not in self.dropper_state.keys():
            print(f"[ERROR] Invalid ball number: {ball_num}")
            return

        # Drop the ball by setting the PWM value for the dropper
        self.set_pwm(*self.dropper_state[ball_num])
        print(f"[INFO] Dropped ball #{ball_num}")

    def reload(self):
        """Sets reloaded to True after the user has manually reloaded the dropper and inputted 'y'"""
        reloaded = False
        while not reloaded:
            inp = input("Reloaded? (y/n): ")
            if inp == "y":
                reloaded = True

        self.state = 0


class Gripper(Polulu):
    """
    Gripper class for the gripper servo

    Inherits:
        Polulu: Class for controlling the servo
    
    Attributes:
        state (int): The state of the gripper

    /!\ Please test this code for gripper, the 0.5 seconds will likely need to be adjusted
    If gripper keeps pushing after reaching its limit, it WILL KILL our power distribution board
    """

    def __init__(self):
        """Initialize the gripper object and set the state of the gripper"""
        super().__init__()

        # state == 0: stop (default)
        # state == 1: open
        # state == 2: close
        self.state = 0

    def open(self):
        """Opens the gripper"""

        # Ensure that the gripper is not already open
        if self.state == 1:
            return

        # Set the PWM value to open the Gripper (for 0.5 secs)
        start_time = time.time()
        while time.time() - start_time < 0.5:
            self.set_pwm(*self.gripper_state[1])
            time.sleep(0.05)

        # Stop the gripper by setting neutral PWM value (1500)
        self.set_pwm(*self.gripper_state[0])
        self.set_pwm(*self.gripper_state[0])
        self.state = 1

    def close(self):
        """Close the gripper"""

        # Ensure the gripper is not already closed
        if self.state == 2:
            return

        # Set the PWM value to close the gripper (for 0.5 secs)
        start_time = time.time()
        while time.time() - start_time < 0.5:
            self.set_pwm(*self.gripper_state[2])
            time.sleep(0.05)

        # Stop the gripper by setting neutral PWM value (1500)
        self.set_pwm(*self.gripper_state[0])
        self.set_pwm(*self.gripper_state[0])
        self.state = 2

class Servo:
    """
    Servo class for controlling the servos (dropper, gripper, torpedoes)

    Attributes:
        USB: Serial connection object for servo control
        gripState (bool): State of the gripper
        torpedoState (int): Current state of the torpedo
        ballState (int): Current state of the dropper
    """
    def __init__(self):
        """Initializes the servo object, sets default attributes"""
        self.USB = serial.Serial(port=dataFromConfig("polulu"))  # Need to find ID of the Polulu controller
        self.USB.isOpen()
        self.gripState = True
        self.torpedoState = 0
        self.ballState = 0
        self.torpedo(0)
        self.dropper(0)
        print("Initialized...")

    def setPwm(self, channel, target):
        """
        Sets the PWM value to control the servo

        Args:
            channel (int): Channel number of the servo (0 for gripper, 1 for dropper, 2 for torpedo)
            target (int): Target PWM value
        """
        target = target * 4
        lsb = target & 0x7F  # Take the 1st 7 bits (right to left) to get the least significant byte
        msb = (target >> 7) & 0x7F  # Shift 7 places to the right and obtain the 1st 7 bits(right to left) to get the most significant byte
        cmd = chr(0x84) + chr(channel) + chr(lsb) + chr(msb) # Encode the command string
        self.USB.write(bytes(cmd.encode())) # Write the command string to the controller

    # please test this code for gripper, the 1.5 seconds will likely need to be adjusted (should be close though)
    # if gripper keeps pushing after reaching its limit, it WILL KILL our power distribution board
    
    def gripper(self, state):
        """
        Controls the gripper mechanism:
        
        Args:
            state (bool): The state of the gripper (true to open, false to close)
        """
        # To ensure the states are not already the same
        if self.gripState == state:
            return
        # If state == True, open the gripper by setting the PWM to 1550, else close the gripper by setting it to 1450
        pwm = 1500
        if state:
            pwm = 1550
        else:
            pwm = 1450
        startTime = time.time()
        # Run for 1.5 seconds
        while time.time() - startTime < 1.5:  
            self.setPwm(0, pwm)
            time.sleep(0.05)
        # Stop the gripper
        self.setPwm(0, 1500) 
        self.setPwm(0, 1500) # redundant
        self.gripState = state

    def torpedo(self, torpedoNum=-1):  
        """
        Controls the firing of the torpedoes

        Args:
            torpedoNum (int): The number of the torpedo to fire (0 : load, 1 : fire #1, 2: fire #2) -> sets default to load state (torpedo)
        """
        # Load state
        if torpedoNum == 0:
            self.setPwm(2, 2400)
            self.torpedoState = 0
            print("Torpedo in Load State")
        # Fire torpedo 1 by setting PWM value to 1700
        elif torpedoNum == 1:  
            self.setPwm(2, 1700)
            print("Torpedo 1 fired")
        # Fire torpedo 2 by setting PWM value to 1300
        elif torpedoNum == 2:  # fire torpedo 2
            self.torpedo(1)  # In case the first torpedo has not been fired
            time.sleep(0.5)
            self.setPwm(2, 1300)
            print("Torpedo 2 fired")
            time.sleep(1)
            self.torpedo(0)  # Reset to load position now that all torpedoes have been fired
        elif torpedoNum == -1:
            self.torpedoState += 1
            self.torpedo(self.torpedoState)
        else:
            print("Invalid Arg in [torpedoLauncher]")

    def dropper(self, ball=-1):  
        """
        Controls the dropper servo

        Args:
            ball (int): The ball to drop (0: load, 1: ball 1, 2: ball 2) -> default is load state
        """

        # Load the balls
        if ball == 0:
            self.setPwm(1, 1600)
            self.ballState = 0
        # Drop the first ball by setting the PWM to 1200
        elif ball == 1: 
            self.setPwm(1, 1200)
        # Drop the second ball by setting the PWM to 700
        elif ball == 2: 
            self.dropper(1)  # In case dropper(1) has not been called before -- this allows for both balls to be dropped without collision
            time.sleep(0.5)
            self.setPwm(1, 700)
            time.sleep(1)
            self.dropper(0)  # Reset to load position now that the dropper is empty
        # Default state is load
        elif ball == -1:
            self.ballState += 1
            self.dropper(self.ballState)
        else:
            print("Invalid Arg in [dropper]")