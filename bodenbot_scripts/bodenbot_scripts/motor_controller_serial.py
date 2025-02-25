#!/usr/bin/env python3
import serial

ser = serial.Serial('/dev/ttyACM0', 115200)


def write_motor_value(motor, speed):
    command = f'write {motor} {speed:.5f}\n'.encode()
    print(command)
    ser.reset_input_buffer()
    ser.write(command)
    response = ser.readline().decode().strip()
    print(response)


def read_motor_value(motor) -> float:
    command = f'read {motor}\n'.encode()
    print(command)
    ser.reset_input_buffer()
    ser.write(command)

    try:
        response = ser.readline().decode().strip()
        print(response)
        value = int(response)
        return value
    except ValueError as e:
        print(e)
        return 0


if __name__ == '__main__':
    write_motor_value(1, 0.5)
    write_motor_value(2, 0.5)
