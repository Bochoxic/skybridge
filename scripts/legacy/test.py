#!/usr/bin/env python3

import math
import time
from pymavlink import mavutil


PORT = "/dev/ttyUSB0"
BAUD = 460800

RC_RATE_HZ = 50.0
ATTITUDE_RATE_HZ = 50.0

YAW_CENTER = 1500
YAW_AMPLITUDE = 300
YAW_FREQUENCY_HZ = 0.2


print(f"Opening {PORT} @ {BAUD}...")

mav = mavutil.mavlink_connection(
    PORT,
    baud=BAUD,
    source_system=255,
    source_component=190,
)


def send_gcs_heartbeat():
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def request_attitude(rate_hz):
    interval_us = int(1_000_000 / rate_hz)

    mav.mav.command_long_send(
        1,  # target_system
        1,  # target_component
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def send_rc_override(roll, pitch, throttle, yaw):
    """
    Send RC_CHANNELS_OVERRIDE.

    Channels:
        1 = Roll
        2 = Pitch
        3 = Throttle
        4 = Yaw

    65535 means "ignore / don't change".
    """

    msg = mavutil.mavlink.MAVLink_rc_channels_override_message(
        1,          # target_system
        1,          # target_component

        roll,       # chan1_raw
        pitch,      # chan2_raw
        throttle,   # chan3_raw
        yaw,        # chan4_raw

        65535,      # chan5
        65535,      # chan6
        65535,      # chan7
        65535,      # chan8
        65535,      # chan9
        65535,      # chan10
        65535,      # chan11
        65535,      # chan12
        65535,      # chan13
        65535,      # chan14
        65535,      # chan15
        65535,      # chan16
        65535,      # chan17
        65535,      # chan18
    )

    mav.mav.send(msg)


# ------------------------------------------------------------
# Startup
# ------------------------------------------------------------

send_gcs_heartbeat()

time.sleep(0.1)

request_attitude(ATTITUDE_RATE_HZ)

print()
print("MAVLink connection started")
print(f"RC rate:        {RC_RATE_HZ:.1f} Hz")
print(f"ATTITUDE rate:  {ATTITUDE_RATE_HZ:.1f} Hz")
print()
print("RC:")
print("  Roll     = 1500")
print("  Pitch    = 1500")
print("  Throttle = 1000")
print(f"  Yaw      = 1500 + {YAW_AMPLITUDE}*sin(2*pi*{YAW_FREQUENCY_HZ}*t)")
print()
print("Yaw range: 1200 ... 1800")
print("Yaw period: 5 seconds")
print()
print("Starting in 2 seconds...")
time.sleep(2.0)


# ------------------------------------------------------------
# Main loop
# ------------------------------------------------------------

start_time = time.monotonic()

next_rc = start_time
next_heartbeat = start_time

last_attitude = None

try:

    while True:

        now = time.monotonic()

        # ----------------------------------------------------
        # GCS heartbeat
        # ----------------------------------------------------

        if now >= next_heartbeat:

            send_gcs_heartbeat()

            next_heartbeat += 1.0

            # Prevent accumulated timing error
            if next_heartbeat < now:
                next_heartbeat = now + 1.0

        # ----------------------------------------------------
        # RC command
        # ----------------------------------------------------

        if now >= next_rc:

            t = now - start_time

            yaw = int(
                YAW_CENTER
                + YAW_AMPLITUDE
                * math.sin(2.0 * math.pi * YAW_FREQUENCY_HZ * t)
            )

            send_rc_override(
                roll=1500,
                pitch=1500,
                throttle=1000,
                yaw=yaw,
            )

            next_rc += 1.0 / RC_RATE_HZ

            # Prevent accumulated timing error
            if next_rc < now:
                next_rc = now + 1.0 / RC_RATE_HZ

            print(
                f"\r"
                f"RC yaw={yaw:4d} "
                f"| roll=1500 "
                f"| pitch=1500 "
                f"| throttle=1000",
                end="",
                flush=True,
            )

        # ----------------------------------------------------
        # Receive MAVLink
        # ----------------------------------------------------

        msg = mav.recv_match(blocking=False)

        if msg is not None:

            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":

                print(
                    f"\nAircraft heartbeat: "
                    f"sys={msg.get_srcSystem()} "
                    f"comp={msg.get_srcComponent()}"
                )

            elif msg_type == "COMMAND_ACK":

                if (
                    msg.command
                    == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
                ):

                    try:
                        result_name = mavutil.mavlink.enums[
                            "MAV_RESULT"
                        ][msg.result].name
                    except Exception:
                        result_name = str(msg.result)

                    print(
                        f"\nATTITUDE rate command ACK: "
                        f"{result_name}"
                    )

            elif msg_type == "ATTITUDE":

                last_attitude = msg

                print(
                    f"\nATTITUDE "
                    f"roll={math.degrees(msg.roll):+7.2f}° "
                    f"pitch={math.degrees(msg.pitch):+7.2f}° "
                    f"yaw={math.degrees(msg.yaw):+7.2f}° "
                    f"| "
                    f"p={math.degrees(msg.rollspeed):+6.1f}°/s "
                    f"q={math.degrees(msg.pitchspeed):+6.1f}°/s "
                    f"r={math.degrees(msg.yawspeed):+6.1f}°/s"
                )

        # Don't burn 100% CPU
        time.sleep(0.0005)


except KeyboardInterrupt:

    print("\n\nStopping...")

    # Put RC channels in a safe state before exiting.
    # Throttle remains at minimum.
    send_rc_override(
        roll=1500,
        pitch=1500,
        throttle=1000,
        yaw=1500,
    )

    time.sleep(0.1)

    print("RC set to neutral/minimum.")