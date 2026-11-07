#!/usr/bin/env python3
"""
Read MAVLink telemetry from the ExpressLRS TX Backpack over WiFi.

The backpack pushes MAVLink as UDP datagrams to port 14550, so we just
bind and listen -- no serial port, no connection handshake.

Setup:
  1. ELRS Lua script: Backpack -> Telemetry -> WiFi
  2. Join WiFi "ExpressLRS TX Backpack XXXXXX"  (password: expresslrs)
     (or leave the backpack in home-WiFi mode and stay on your own LAN)
  3. python3 wifi_telem.py

Usage:
    python3 wifi_telem.py [port]
"""

import math
import sys
import threading
import time
from pymavlink import mavutil

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 14550

# The backpack SENDS to 14550 but LISTENS on a different port (14555 by
# default, see "MavLink Configuration" in its WebUI). Heartbeats must go
# to the listen port or it never learns our IP and keeps broadcasting.
ANNOUNCE_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 14555

# The backpack broadcasts only until a GCS talks to it, then it latches
# onto that IP (gcsIP/gcsIPSet in devWIFI.cpp). So we poke it with our own
# heartbeats -- that both announces us and keeps the latch fresh.
BACKPACK = "elrs_txbp.local"

# We manage the socket ourselves rather than using pymavlink's udpin:
# the backpack sends FROM port 14555 TO our 14550, and pymavlink's udpin
# locks onto a single peer address, which drops those packets.
CONN = None

print(f"Listening for MAVLink on UDP {PORT} ...")
print("(ELRS backpack AP is 10.0.0.1 -- make sure you're on its WiFi)")
print()

import socket

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
_sock.bind(("0.0.0.0", PORT))

# A parser we feed by hand from the raw socket.
mav = mavutil.mavlink.MAVLink(None)
mav.robust_parsing = True


class _Link:
    """Minimal shim so the rest of the script reads like a connection."""

    target_system = 1
    target_component = 1

    def recv_msgs(self, timeout=1.0):
        _sock.settimeout(timeout)
        try:
            data, _addr = _sock.recvfrom(4096)
        except socket.timeout:
            return []
        try:
            return mav.parse_buffer(data) or []
        except Exception:
            return []


link = _Link()

# Announce ourselves to the backpack so it learns our IP and stops
# relying on broadcast (which some APs drop).
def announce():
    try:
        out = mavutil.mavlink_connection(
            f"udpout:{BACKPACK}:{ANNOUNCE_PORT}",
            source_system=255,
            source_component=190,
        )
    except Exception as e:
        print(f"(announce disabled: {e})")
        return
    while True:
        try:
            out.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
            )
        except Exception:
            pass
        time.sleep(1.0)

threading.Thread(target=announce, daemon=True).start()
print(f"Announcing to {BACKPACK}:{ANNOUNCE_PORT} so it learns our IP...")

# Wait for the first heartbeat so we learn the vehicle's sysid.
print("Waiting for heartbeat...")

# Betaflight over ELRS may not emit HEARTBEAT; accept ANY message as
# proof of life and take the sysid from it.
hb = None
_deadline = time.monotonic() + 30
while time.monotonic() < _deadline:
    for m in link.recv_msgs(1.0):
        if m.get_type() == "BAD_DATA":
            continue
        hb = m
        link.target_system = m.get_srcSystem()
        link.target_component = m.get_srcComponent()
        print(f"First message: {m.get_type()}")
        break
    if hb is not None:
        break

if hb is None:
    print()
    print("No heartbeat in 30s. Check:")
    print("  - Connected to the backpack's WiFi?")
    print("  - Lua: Backpack -> Telemetry -> WiFi ?")
    print("  - Firewall blocking inbound UDP? (sudo ufw allow 14550/udp)")
    print("  - Is the drone powered and linked (TX not in a failsafe)?")
    print("  - Backpack firmware must be MAVLINK-enabled (generic TX Backpack,")
    print("    not the DupleTX/vendor build) and TX must be in MAVLink mode.")
    sys.exit(1)

print(f"Heartbeat from system {link.target_system} component {link.target_component}")
print()

# Ask for ATTITUDE at 50 Hz, same as test.py did over serial.
_req = mavutil.mavlink.MAVLink(None)
_req.srcSystem = 255
_req.srcComponent = 190
_msg = _req.command_long_encode(
    link.target_system,
    link.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    int(1_000_000 / 50),   # interval in microseconds
    0, 0, 0, 0, 0,
)
try:
    _sock.sendto(_msg.pack(_req), (BACKPACK, ANNOUNCE_PORT))
except Exception as e:
    print(f"(attitude request failed: {e})")

counts = {}
last_report = time.monotonic()
hz = 0.0

try:
    while True:
        batch = link.recv_msgs(1.0)

        # One UDP datagram carries several MAVLink messages (the backpack
        # batches them before sending). Printing every one just replays
        # stale frames, so count them all but display only the newest.
        latest_attitude = None

        for msg in batch:

            mtype = msg.get_type()

            if mtype == "BAD_DATA":
                continue

            counts[mtype] = counts.get(mtype, 0) + 1

            if mtype == "ATTITUDE":
                latest_attitude = msg

        if latest_attitude is not None:
            msg = latest_attitude
            # Live single-line readout. \r keeps it on one line and
            # flush=True pushes it out immediately instead of waiting
            # for the buffer to fill.
            print(
                f"\rroll={math.degrees(msg.roll):+8.2f}deg  "
                f"pitch={math.degrees(msg.pitch):+8.2f}deg  "
                f"yaw={math.degrees(msg.yaw):+8.2f}deg  "
                f"| p={math.degrees(msg.rollspeed):+7.1f}  "
                f"q={math.degrees(msg.pitchspeed):+7.1f}  "
                f"r={math.degrees(msg.yawspeed):+7.1f} deg/s  "
                f"| {hz:4.1f}Hz"
                f"\033[K",          # clear to end of line
                end="",
                flush=True,
            )

        # Recompute the rate once a second, but never print a newline --
        # the value is shown inline on the single live readout above.
        now = time.monotonic()
        if now - last_report >= 1.0:
            elapsed = now - last_report
            hz = counts.get("ATTITUDE", 0) / elapsed
            counts.clear()
            last_report = now

except KeyboardInterrupt:
    print("\nStopped.")
