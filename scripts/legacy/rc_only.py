#!/usr/bin/env python3

import time
from pymavlink import mavutil

PORT = "/dev/ttyUSB0"
BAUD = 460800

mav = mavutil.mavlink_connection(
    PORT,
    baud=BAUD,
    source_system=255,
    source_component=190,
)

def send_rc():
    msg = mavutil.mavlink.MAVLink_rc_channels_override_message(
        1,      # target system
        1,      # target component

        1500,   # CH1 roll
        1500,   # CH2 pitch
        1000,   # CH3 throttle
        1500,   # CH4 yaw

        65535,
        65535,
        65535,
        65535,
        65535,
        65535,
        65535,
        65535,
        65535,
        65535,
        65535,
        65535,
        65535,
        65535,
    )

    mav.mav.send(msg)


print("Sending fixed RC override")
print("Roll     = 1500")
print("Pitch    = 1500")
print("Throttle = 1500")
print("Yaw      = 1500")
print()
print("Rate = 100 Hz")
print()
print("STOP WITH CTRL+C")
print()

next_send = time.monotonic()

try:

    RC_RATE = 100.0

    next_send = time.monotonic()

    while True:
        now = time.monotonic()

        if now >= next_send:

            send_rc()

            next_send += 1.0 / RC_RATE

        time.sleep(0.001)

except KeyboardInterrupt:
    print("\nStopped.")