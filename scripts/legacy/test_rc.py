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

print("Connected")

# GCS heartbeat
mav.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0,
    0,
    mavutil.mavlink.MAV_STATE_ACTIVE,
)

print("Sending yaw = 1700 at 5 Hz")

while True:
    msg = mavutil.mavlink.MAVLink_rc_channels_override_message(
        1,      # target_system
        1,      # target_component
        1500,   # chan1 roll
        1500,   # chan2 pitch
        1000,   # chan3 throttle
        1700,   # chan4 yaw
        0,      # chan5
        0,      # chan6
        0,      # chan7
        0,      # chan8
    )

    mav.mav.send(msg)

    print("sent yaw=1700")
    time.sleep(0.2)