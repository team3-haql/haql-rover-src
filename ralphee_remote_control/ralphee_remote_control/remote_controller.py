import inputs
import threading
import math
from enum import IntEnum
import os
from time import sleep, time
import importlib

DEADZONE = 0.025
DEADZONE_INV = 1-DEADZONE

MAX_SPEED = 0.75

# Used as reference
# https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python
class Controller(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.velocity: float = 0.0
        self.angle: float = math.nan

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self.last_update_time = time()

        self._monitor_thread = threading.Thread(target=self.update, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def update(self):
        while True:
            self.last_update_time = time()
            events = None
            try:
                events = inputs.get_gamepad()
            except:
                print('[update_inputs] not connected!')
                importlib.reload(inputs)
                self.velocity = 0
                self.angle = 0
                sleep(1)
                continue
            
            self.update_inputs(events)
            if self.B and self.A and self.X and self.Y:
                print('shutdown!')
                os.system("sudo shutdown") 
            self.update_velocity()  
            self.update_angle()

    def update_inputs(self, events):
        for event in events:
            if event.code == 'ABS_Y':
                self.LeftJoystickY = event.state / Controller.MAX_JOY_VAL # normalize between -1 and 1
            elif event.code == 'ABS_X':
                self.LeftJoystickX = event.state / Controller.MAX_JOY_VAL # normalize between -1 and 1
            elif event.code == 'ABS_RY':
                self.RightJoystickY = event.state / Controller.MAX_JOY_VAL # normalize between -1 and 1
            elif event.code == 'ABS_RX':
                self.RightJoystickX = event.state / Controller.MAX_JOY_VAL # normalize between -1 and 1
            elif event.code == 'ABS_Z':
                self.LeftTrigger = event.state / Controller.MAX_TRIG_VAL # normalize between 0 and 1
            elif event.code == 'ABS_RZ':
                self.RightTrigger = event.state / Controller.MAX_TRIG_VAL # normalize between 0 and 1
            elif event.code == 'BTN_TL':
                self.LeftBumper = event.state
            elif event.code == 'BTN_TR':
                self.RightBumper = event.state
            elif event.code == 'BTN_SOUTH':
                self.A = event.state
            elif event.code == 'BTN_NORTH':
                self.Y = event.state #previously switched with X
            elif event.code == 'BTN_WEST':
                self.X = event.state #previously switched with Y
            elif event.code == 'BTN_EAST':
                self.B = event.state
            elif event.code == 'BTN_THUMBL':
                self.LeftThumb = event.state
            elif event.code == 'BTN_THUMBR':
                self.RightThumb = event.state
            elif event.code == 'BTN_SELECT':
                self.Back = event.state
            elif event.code == 'BTN_START':
                self.Start = event.state
            elif event.code == 'BTN_TRIGGER_HAPPY1':
                self.LeftDPad = event.state
            elif event.code == 'BTN_TRIGGER_HAPPY2':
                self.RightDPad = event.state
            elif event.code == 'BTN_TRIGGER_HAPPY3':
                self.UpDPad = event.state
            elif event.code == 'BTN_TRIGGER_HAPPY4':
                self.DownDPad = event.state

    def update_velocity(self):
        # Equation https://www.desmos.com/calculator/hmsh6km1tl
        if abs(self.LeftJoystickY) < DEADZONE:
            self.velocity = 0.0
            return
        # Keeps joystick in 'square' zone
        if self.LeftJoystickY != 0.0:
            sign = abs(self.LeftJoystickY)/self.LeftJoystickY
        else:
            sign = 1.0
        vel = (self.LeftJoystickY-DEADZONE*sign) / (DEADZONE_INV)

        speed_multiplier = MAX_SPEED
        if self.LeftTrigger:
            speed_multiplier *= 2
        if self.LeftBumper:
            speed_multiplier *= 0.5

        self.velocity = max(-1, min(1, vel)) * speed_multiplier

    def update_angle(self):
        # Equation https://www.desmos.com/calculator/hmsh6km1tl
        
        if abs(self.RightJoystickX) < DEADZONE:
            self.angle = 0.0
            return
        # Keeps joystick in 'square' zone
        if self.RightJoystickX != 0.0:
            sign = abs(self.RightJoystickX)/self.RightJoystickX
        else:
            sign = 1.0
        angle = (self.RightJoystickX - DEADZONE*sign) / (DEADZONE_INV)
        angle_multiplier = 0.75
        if self.RightTrigger:
            angle_multiplier = 1
        if self.RightBumper:
            angle_multiplier = 0.5
        self.angle = max(-1, min(1, angle*angle_multiplier))