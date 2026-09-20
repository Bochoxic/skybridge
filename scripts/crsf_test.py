#!/usr/bin/env python3
"""
CRSF drone controller + telemetry printer.

Sends RC channels at 50 Hz and decodes every CRSF telemetry frame
received from the ELRS TX module, printing values in real units.

Usage:
    python3 crsf_drone.py [/dev/ttyUSB0] [115200]
"""

import math
import struct
import sys
import time

import serial

# ── Serial ────────────────────────────────────────────────────────────────────
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

# ── CRSF constants ────────────────────────────────────────────────────────────
ADDR_MODULE     = 0xC8
TYPE_RC_CHANNELS = 0x16

CH_MIN = 172
CH_MID = 992
CH_MAX = 1811

# ── Helpers ───────────────────────────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def pack_channels(channels) -> bytes:
    bits = nbits = 0
    out = bytearray()
    for ch in channels:
        bits |= (ch & 0x7FF) << nbits
        nbits += 11
        while nbits >= 8:
            out.append(bits & 0xFF)
            bits >>= 8
            nbits -= 8
    if nbits:
        out.append(bits & 0xFF)
    return bytes(out)


def rc_frame(channels) -> bytes:
    payload = pack_channels(channels)
    body = bytes([TYPE_RC_CHANNELS]) + payload
    return bytes([ADDR_MODULE, len(body) + 1]) + body + bytes([crc8(body)])


def us_to_crsf(us: float) -> int:
    us = max(1000.0, min(2000.0, us))
    return int(CH_MIN + (us - 1000) / 1000 * (CH_MAX - CH_MIN))


# ── CRSF frame parser ─────────────────────────────────────────────────────────
def parse_frames(buf: bytearray):
    """Yield (frame_type, payload_bytes) from buf, consuming valid frames."""
    while len(buf) >= 4:
        length = buf[1]
        if length < 2 or length > 62:
            buf.pop(0)
            continue
        total = length + 2
        if len(buf) < total:
            return
        frame = bytes(buf[:total])
        del buf[:total]
        body = frame[2:-1]
        if crc8(body) == frame[-1]:
            yield body[0], body[1:]


# ── Telemetry decoders ────────────────────────────────────────────────────────
def decode_link_stats(p):
    if len(p) < 10:
        return {}
    return {
        "uplink_rssi1":   -p[0],           # dBm
        "uplink_rssi2":   -p[1],           # dBm
        "uplink_lq":       p[2],           # %
        "uplink_snr":      struct.unpack_from("b", p, 3)[0],  # dB signed
        "rf_mode":         p[4],
        "uplink_power":    p[5],
        "downlink_rssi":  -p[6],           # dBm
        "downlink_lq":     p[7],           # %
        "downlink_snr":    struct.unpack_from("b", p, 8)[0],  # dB signed
    }


def decode_battery(p):
    if len(p) < 8:
        return {}
    voltage_raw, current_raw = struct.unpack_from(">HH", p, 0)
    mah_used = (p[4] << 16 | p[5] << 8 | p[6])
    remaining = p[7]
    return {
        "voltage_V":    voltage_raw / 10.0,
        "current_A":    current_raw / 10.0,
        "capacity_mAh": mah_used,
        "remaining_%":  remaining,
    }


def decode_attitude(p):
    if len(p) < 6:
        return {}
    pitch_raw, roll_raw, yaw_raw = struct.unpack_from(">hhh", p, 0)
    return {
        "pitch_deg": round(math.degrees(pitch_raw / 10000.0), 2),
        "roll_deg":  round(math.degrees(roll_raw  / 10000.0), 2),
        "yaw_deg":   round(math.degrees(yaw_raw   / 10000.0), 2),
    }


def decode_flight_mode(p):
    try:
        return {"mode": p.rstrip(b"\x00").decode("ascii", errors="replace")}
    except Exception:
        return {}


def decode_baro_alt(p):
    if len(p) < 2:
        return {}
    alt_raw = struct.unpack_from(">H", p, 0)[0]
    # units: dm if < 10000, else (val - 10000) * 10 m
    if alt_raw < 10000:
        alt_m = alt_raw / 10.0
    else:
        alt_m = (alt_raw - 10000) * 10.0
    vario = 0.0
    if len(p) >= 4:
        vario = struct.unpack_from(">h", p, 2)[0] / 100.0  # m/s
    return {"alt_m": alt_m, "vario_ms": vario}


# New AHRS frames added in Betaflight 2026.6
def decode_ahrs_accel(p):
    if len(p) < 6:
        return {}
    ax, ay, az = struct.unpack_from(">hhh", p, 0)
    # raw ADC counts — scale depends on gyro config; print raw for now
    return {"acc_x_raw": ax, "acc_y_raw": ay, "acc_z_raw": az}


def decode_ahrs_gyro(p):
    if len(p) < 6:
        return {}
    gx, gy, gz = struct.unpack_from(">hhh", p, 0)
    return {"gyro_x_raw": gx, "gyro_y_raw": gy, "gyro_z_raw": gz}


DECODERS = {
    0x08: ("BATTERY",     decode_battery),
    0x09: ("BARO_ALT",    decode_baro_alt),
    0x14: ("LINK_STATS",  decode_link_stats),
    0x1E: ("ATTITUDE",    decode_attitude),
    0x21: ("FLIGHT_MODE", decode_flight_mode),
    # AHRS frames — type IDs may differ; will print unknown ones with raw bytes
    0x2E: ("ELRS_STATUS", lambda p: {"raw": p.hex()}),
    0x3A: ("ELRS_LINK",   lambda p: {"raw": p.hex()}),
}


# Latest value of each frame type, shown together on one live line.
_latest: dict[str, str] = {}


def record(ftype, payload):
    """Decode a frame and stash its latest values. No printing."""
    if ftype == 0x3A:
        return

    if ftype in DECODERS:
        name, decoder = DECODERS[ftype]
        values = decoder(payload)

        if name == "ATTITUDE":
            _latest["attitude"] = values
        elif name == "LINK_STATS":
            # Link quality explains why the telemetry rate moves around:
            # telemetry only gets the airtime left over from RC packets.
            _latest["lq"] = values.get("uplink_lq", 0)
            _latest["rssi"] = values.get("uplink_rssi1", 0)
        else:
            _latest[name] = " ".join(f"{k}={v}" for k, v in values.items())
    else:
        _latest[f"0x{ftype:02X}"] = payload.hex()


def render(hz):
    """Draw the single live line from whatever we last recorded."""
    att = _latest.get("attitude")
    if att is None:
        return

    extra = "  ".join(
        v for k, v in _latest.items()
        if k not in ("attitude", "lq", "rssi") and isinstance(v, str)
    )

    print(
        f"\rroll={att.get('roll_deg', 0):+8.2f}deg  "
        f"pitch={att.get('pitch_deg', 0):+8.2f}deg  "
        f"yaw={att.get('yaw_deg', 0):+8.2f}deg  "
        f"| {hz:4.1f}Hz  "
        f"LQ={_latest.get('lq', 0):3d}%  "
        f"RSSI={_latest.get('rssi', 0):4d}dBm  "
        f"{extra}"
        f"\033[K",
        end="",
        flush=True,
    )


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ser = serial.Serial(PORT, BAUD, timeout=0)
    print(f"Opened {PORT} @ {BAUD} baud")
    print("Sending RC at 50 Hz  |  Throttle=MIN  |  ARM=OFF")
    print("Telemetry frames printed as they arrive\n")

    buf         = bytearray()
    start       = time.monotonic()
    last_rc     = start
    last_render = start
    last_rate   = start
    att_count   = 0
    hz          = 0.0

    try:
        while True:
            t = time.monotonic() - start

            # ── Build RC channels ──────────────────────────────────────────
            sine   = math.sin(2 * math.pi * 0.25 * t)   # 0.25 Hz sine
            yaw_us = 1500 + 500 * sine

            channels    = [CH_MID] * 16
            channels[0] = CH_MID                   # roll
            channels[1] = CH_MID                   # pitch
            channels[2] = CH_MIN                   # throttle LOW
            channels[3] = us_to_crsf(yaw_us)       # yaw sine
            channels[4] = CH_MIN                   # AUX1 ARM OFF
            channels[5] = CH_MIN                   # AUX2

            ser.write(rc_frame(channels))

            # ── Read and decode telemetry ──────────────────────────────────
            data = ser.read(512)
            if data:
                buf.extend(data)
                for ftype, payload in parse_frames(buf):
                    record(ftype, payload)
                    if ftype == 0x1E:
                        att_count += 1
                        render(hz)

            now = time.monotonic()

            # ── Measured ATTITUDE rate, once a second ──────────────────────
            if now - last_rate >= 1.0:
                hz = att_count / (now - last_rate)
                att_count = 0
                last_rate = now

            time.sleep(0.02)   # 50 Hz

    except KeyboardInterrupt:
        ser.close()
        print("\nClosed.")


if __name__ == "__main__":
    main()