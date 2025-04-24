import serial
import time
import os.path

async def init_servos() -> serial.Serial:
    print('[init_servos] create arduino object')
    # Check for existence of file
    if not os.path.exists('/dev/arduino'):
        raise Exception('[init_servos] arduino not plugged in!')
    
    # Creates arduino object
    arduino = serial.Serial('/dev/arduino', 9600)

    # Sets up usb port
    print('[init_servos] setup serial port for arduino')
    arduino.setDTR(False)
    time.sleep(0.1) # Initially set to 1
    arduino.flushInput()
    arduino.setDTR(True)
    time.sleep(0.1) # Initially set to 2
    print('[init_servos] arduino ready!')
    return arduino

async def update_servos(angle: float, arduino: serial.Serial):
    arduino.write((str(angle) + '\r').encode())
    # print(f'[update_servos] t: {angle}')