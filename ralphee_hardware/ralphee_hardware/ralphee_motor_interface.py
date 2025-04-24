import moteus
import time
import math
import usb
import numpy as np

# Can bus id, bisit mjbots moteus repo for more info: https://github.com/mjbots/moteus
# center motor id should be in center.
# 1 5 3
# 0 3
LEFT_MOTOR_IDS  = [2, 5, 3]
RIGHT_MOTOR_IDS = [0, 4, 1]

# [0] = Left Non Center
# [1] = Left Center
# [2] = Right Non Center
# [3] = Right Center
MOTOR_COORDS = np.array([[-0.3048,0.3556], [-0.3048,0.0],  # Left Side
                         [ 0.3048,0.3556], [ 0.3048,0.0]]) # Right Side

def get_motor_radiuses(radius: float) -> np.ndarray[float]:
    """
        Gets radius to each motor, used velocity calculation
        Args:
            radius:
                Radius from center over rover to turn point
        Return:
            Array of radius to each motor
    """
    global MOTOR_COORDS

    ROVER_CENTER = np.array([radius,0.0])
    radiuses = np.empty(6, dtype=float)

    for i, motor_coord in enumerate(MOTOR_COORDS):
        p = ROVER_CENTER - motor_coord
        sum_sq = np.dot(p, p)
        radiuses[i] = np.sqrt(sum_sq)

    return radiuses

async def init_motors() -> list[list[moteus.Controller]]:
    """
        Initializes moteus controllers!
        Return:
            Groups of moteus controllers! Each group has a different velocity sent to it.
    """
    print("[init_motors] resetting device file...")

    # Reset fdcamusb device
    # Without this step the program will get stuck at 'set_stop' on repeat runs.
    dev = usb.core.find(idVendor=0x0483, idProduct=0x5740)
    if dev is None:
        raise Exception('[init_motors] plug in motors!')

    dev.reset()

    print("[init_motors] reset usb!")
    print('[init_motors] create controller objects')
    qr = moteus.QueryResolution()
    qr.trajectory_complete = moteus.INT8

    # Create Motor Objects
    left_controllers = []
    for id in LEFT_MOTOR_IDS:
        left_controllers.append(moteus.Controller(id, query_resolution=qr))
    right_controllers = []
    for id in RIGHT_MOTOR_IDS:
        right_controllers.append(moteus.Controller(id, query_resolution=qr))

    # Reset motors incase of inpropper shutdown.
    print('[init_motors] set stop')
    for c in left_controllers:
        await c.set_stop()
    for c in right_controllers:
        await c.set_stop()
    print('[init_motors] ready!')

    # Return groups
    return [left_controllers, right_controllers]

async def update_motors(velocity: float, radius: float, controller_groups: list[list[moteus.Controller]]):
    """
        Updates motor velocities. https://www.desmos.com/calculator/pwxu4jlu2f
        Args:
            velocity:
                Speed that the rover itself will travel at.
            radius:
                length from turning point to center of body
            controller_groups:
                Controller groups to be set.
    """
    coroutines = []

    if math.isfinite(radius):
        radiuses = get_motor_radiuses(radius) / radius
    else:
        radiuses = np.ones(4)

    # Gets the velocity for both sides.
    left_velocity         =  velocity*abs(radiuses[0])
    left_center_velocity  =  velocity*abs(radiuses[1])
    right_velocity        = -velocity*abs(radiuses[2])
    right_center_velocity = -velocity*abs(radiuses[3])

    LEFT_SIDE_INDEX  = 0
    RIGHT_SIDE_INDEX = 1

    # Start coroutines
    for i, c in enumerate(controller_groups[LEFT_SIDE_INDEX]):
        v = left_velocity if i != 1 else left_center_velocity
        coroutines.append(c.set_position(position=math.nan, velocity=v, query=True, watchdog_timeout=10.0))

    for i, c in enumerate(controller_groups[RIGHT_SIDE_INDEX]):
        v = right_velocity if i != 1 else right_center_velocity
        coroutines.append(c.set_position(position=math.nan, velocity=v, query=True, watchdog_timeout=10.0))

    # print(f'[update_motors] lv: {left_velocity}, lcv: {left_center_velocity}, rv: {right_velocity}, rcv: {right_center_velocity}')

    # Await coroutines
    for coroutine in coroutines:
        await coroutine