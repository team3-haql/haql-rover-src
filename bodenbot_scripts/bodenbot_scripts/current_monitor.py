#!/usr/bin/env python3
from .ina233 import INA233

R_SHUNT_OHMS = 0.0025
I_MAX_AMPS = 10

bus = 1

addresses = [
    0x40,
    0x42,
    0x43,
    0x4A,
    0x4B,
    0x4F,
]

devices = []

for address in addresses:
    device = INA233(bus, address)
    device.calibrate(R_SHUNT_OHMS, I_MAX_AMPS)
    devices.append((device, address))


def print_readings():
    for device, address in devices:
        print(f"[0x{address:x}] Voltage: {device.getBusVoltageIn_V():.3f} V")
        print(f"[0x{address:x}] Current: {device.getCurrentIn_mA():.3f} mA")


if __name__ == "__main__":
    print_readings()
