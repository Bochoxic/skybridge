#!/usr/bin/env python3

import sys
import time
from pymavlink import mavutil

RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
PORT = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0"
BAUD = int(sys.argv[3]) if len(sys.argv) > 3 else 460800

print(f"Opening {PORT} @ {BAUD}...")

mav = mavutil.mavlink_connection(
    PORT,
    baud=BAUD,
    source_system=255,
    source_component=190,
)

# ------------------------------------------------------------
# Start the MAVLink session immediately
# ------------------------------------------------------------

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
        1,  # target system
        1,  # target component
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        interval_us,
        0, 0, 0, 0, 0
    )


# Send immediately — DO NOT wait for heartbeat first.
send_gcs_heartbeat()
time.sleep(0.1)

request_attitude(RATE)

print(f"ATTITUDE requested @ {RATE:.1f} Hz")
print("Waiting for ATTITUDE...\n")

# ------------------------------------------------------------
# Runtime
# ------------------------------------------------------------

last_heartbeat = time.monotonic()

t0 = None
count = 0
rate_hz = 0.0

last_attitude_time = None

while True:

    now = time.monotonic()

    # Keep GCS heartbeat alive
    if now - last_heartbeat >= 1.0:
        send_gcs_heartbeat()
        last_heartbeat = now

    # Receive any MAVLink message
    msg = mav.recv_match(
        blocking=False
    )

    if msg is None:
        time.sleep(0.001)
        continue

    msg_type = msg.get_type()

    # --------------------------------------------------------
    # Heartbeat / ACK diagnostics
    # --------------------------------------------------------

    if msg_type == "HEARTBEAT":

        print(
            f"\nAircraft heartbeat: "
            f"sys={msg.get_srcSystem()} "
            f"comp={msg.get_srcComponent()}"
        )

    elif msg_type == "COMMAND_ACK":

        if msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:

            try:
                result_name = (
                    mavutil.mavlink.enums["MAV_RESULT"]
                    [msg.result].name
                )
            except Exception:
                result_name = str(msg.result)

            print(
                f"\nATTITUDE rate command ACK: {result_name}"
            )

    # --------------------------------------------------------
    # ATTITUDE
    # --------------------------------------------------------

    if msg_type == "ATTITUDE":

        now = time.monotonic()

        # Rate measurement
        if t0 is None:
            t0 = now

        count += 1

        elapsed = now - t0

        if elapsed >= 1.0:
            rate_hz = count / elapsed
            count = 0
            t0 = now

        # Actual interval
        if last_attitude_time is None:
            dt_ms = 0.0
        else:
            dt_ms = (
                now - last_attitude_time
            ) * 1000.0

        last_attitude_time = now

        print(
            f"\r"
            f"roll={msg.roll:+7.3f} "
            f"pitch={msg.pitch:+7.3f} "
            f"yaw={msg.yaw:+7.3f} rad "
            f"| "
            f"p={msg.rollspeed:+6.3f} "
            f"q={msg.pitchspeed:+6.3f} "
            f"r={msg.yawspeed:+6.3f} rad/s "
            f"| "
            f"{rate_hz:5.1f} Hz "
            f"| dt={dt_ms:5.1f} ms",
            end="",
            flush=True,
        )