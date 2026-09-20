"""CRSF telemetry frame decoders.

Carried over from the working reference script. Each decoder takes a
payload and returns a dict of values in real units, or an empty dict if
the payload is too short.
"""

from __future__ import annotations

import math
import struct
from collections.abc import Callable

from .constants import (
    FRAME_ATTITUDE,
    FRAME_BARO_ALT,
    FRAME_BATTERY,
    FRAME_ELRS_LINK,
    FRAME_ELRS_STATUS,
    FRAME_FLIGHT_MODE,
    FRAME_LINK_STATS,
)


def decode_link_stats(p: bytes) -> dict:
    if len(p) < 10:
        return {}
    return {
        "uplink_rssi1": -p[0],                               # dBm
        "uplink_rssi2": -p[1],                               # dBm
        "uplink_lq": p[2],                                   # %
        "uplink_snr": struct.unpack_from("b", p, 3)[0],      # dB
        "rf_mode": p[4],
        "uplink_power": p[5],
        "downlink_rssi": -p[6],                              # dBm
        "downlink_lq": p[7],                                 # %
        "downlink_snr": struct.unpack_from("b", p, 8)[0],    # dB
    }


def decode_battery(p: bytes) -> dict:
    if len(p) < 8:
        return {}
    voltage_raw, current_raw = struct.unpack_from(">HH", p, 0)
    mah_used = (p[4] << 16) | (p[5] << 8) | p[6]
    return {
        "voltage_V": voltage_raw / 10.0,
        "current_A": current_raw / 10.0,
        "capacity_mAh": mah_used,
        "remaining_%": p[7],
    }


def decode_attitude(p: bytes) -> dict:
    """Decode attitude.

    Payload order is pitch, roll, yaw (CRSF's order, not roll-first) and
    each value is raw/10000 radians.
    """
    if len(p) < 6:
        return {}
    pitch_raw, roll_raw, yaw_raw = struct.unpack_from(">hhh", p, 0)
    return {
        "pitch_deg": round(math.degrees(pitch_raw / 10000.0), 2),
        "roll_deg": round(math.degrees(roll_raw / 10000.0), 2),
        "yaw_deg": round(math.degrees(yaw_raw / 10000.0), 2),
    }


def decode_flight_mode(p: bytes) -> dict:
    try:
        return {"mode": p.rstrip(b"\x00").decode("ascii", errors="replace")}
    except Exception:
        return {}


def decode_baro_alt(p: bytes) -> dict:
    """Decode barometric altitude.

    Dual scale: raw < 10000 means decimetres, otherwise (raw - 10000) * 10
    metres. Vario is present when the payload is long enough.
    """
    if len(p) < 2:
        return {}
    alt_raw = struct.unpack_from(">H", p, 0)[0]
    if alt_raw < 10000:
        alt_m = alt_raw / 10.0
    else:
        alt_m = (alt_raw - 10000) * 10.0

    out = {"alt_m": alt_m}
    if len(p) >= 4:
        out["vario_ms"] = struct.unpack_from(">h", p, 2)[0] / 100.0
    return out


DECODERS: dict[int, tuple[str, Callable[[bytes], dict]]] = {
    FRAME_BATTERY: ("BATTERY", decode_battery),
    FRAME_BARO_ALT: ("BARO_ALT", decode_baro_alt),
    FRAME_LINK_STATS: ("LINK_STATS", decode_link_stats),
    FRAME_ATTITUDE: ("ATTITUDE", decode_attitude),
    FRAME_FLIGHT_MODE: ("FLIGHT_MODE", decode_flight_mode),
    FRAME_ELRS_STATUS: ("ELRS_STATUS", lambda p: {"raw": p.hex()}),
    FRAME_ELRS_LINK: ("ELRS_LINK", lambda p: {"raw": p.hex()}),
}
