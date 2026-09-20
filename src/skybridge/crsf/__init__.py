"""CRSF protocol: framing and telemetry decoding."""

from .constants import CH_MAX, CH_MID, CH_MIN, NUM_CHANNELS
from .decoders import DECODERS
from .protocol import crc8, pack_channels, parse_frames, rc_frame

__all__ = [
    "CH_MAX",
    "CH_MID",
    "CH_MIN",
    "DECODERS",
    "NUM_CHANNELS",
    "crc8",
    "pack_channels",
    "parse_frames",
    "rc_frame",
]
