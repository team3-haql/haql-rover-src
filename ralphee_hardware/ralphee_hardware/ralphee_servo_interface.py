import serial
import time

async def init_servos() -> serial.Serial:
    """
        Initializes servo arduino serial port.
        Returns:
            Serial port of arduino
    """
    print('[init_servos] create arduino object')
    # Creates arduino object
    arduino = serial.Serial('/dev/ttyACM1', 9600)

    # Without this step you MAY encounter issues setting up arduino.
    print('[init_servos] setup serial port for arduino')
    arduino.setDTR(False)
    time.sleep(0.1) # Initially set to 1
    arduino.flushInput()
    arduino.setDTR(True)
    time.sleep(0.1) # Initially set to 2
    print('[init_servos] arduino ready!')

    return arduino

async def update_servos(angle: float, arduino: serial.Serial):
    """
        Updates angle of servos.
        Args:
            angle:
                Angle that the rover itself is traveling at
            arduino:
                Serial connection to servo arduino
    """
    # Arduino code github: https://github.com/team3-haql/ServoArduinoCode/tree/main
    arduino.write((str(angle) + '\r').encode())
    print(f'[update_servos] t: {angle}')