"""CRSF protocol constants.

Channel tick values and the tick<->microsecond relationship come from
Betaflight's own decoder, `src/main/rx/crsf.c`:

    min  172 ->  988us
    mid  992 -> 1500us
    max 1811 -> 2012us
    us = 0.62477120195241 * tick + 881

Note that 172 is *not* 1000us -- it is 988us. The anchors that land on
round microseconds are 191 / 992 / 1792, which is what `units.py` uses.
"""

# ── Framing ───────────────────────────────────────────────────────────────────
ADDR_MODULE = 0xC8          # sync byte: flight controller / TX module
TYPE_RC_CHANNELS = 0x16     # frame type for packed 16ch RC data

CRC8_POLY = 0xD5

MAX_FRAME_LEN = 62
MIN_FRAME_LEN = 2

# ── Channel tick range ────────────────────────────────────────────────────────
CH_MIN = 172                # 988us
CH_MID = 992                # 1500us
CH_MAX = 1811               # 2012us

NUM_CHANNELS = 16

# Betaflight's affine decode (rx/crsf.c).
US_PER_TICK = 0.62477120195241
US_OFFSET = 881

# ── Telemetry frame types ─────────────────────────────────────────────────────
FRAME_GPS = 0x02
FRAME_BATTERY = 0x08
FRAME_BARO_ALT = 0x09
FRAME_LINK_STATS = 0x14
FRAME_ATTITUDE = 0x1E
FRAME_FLIGHT_MODE = 0x21
FRAME_ELRS_STATUS = 0x2E
FRAME_ELRS_LINK = 0x3A
