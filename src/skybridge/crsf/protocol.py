"""CRSF framing: CRC, channel packing, frame build and parse.

These functions are carried over verbatim from the working reference
script (`scripts/crsf_test.py`) -- they are proven on hardware, so they
are deliberately unchanged apart from type hints and docstrings.
"""

from __future__ import annotations

from collections.abc import Iterator, Sequence

from .constants import (
    ADDR_MODULE,
    CRC8_POLY,
    MAX_FRAME_LEN,
    MIN_FRAME_LEN,
    TYPE_RC_CHANNELS,
)


def crc8(data: bytes) -> int:
    """CRSF CRC-8, poly 0xD5, MSB-first, init 0, no reflection or final XOR.

    Computed over the frame *body* (type byte + payload), not over the
    address or length bytes.
    """
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def pack_channels(channels: Sequence[int]) -> bytes:
    """Pack N x 11-bit channel values into an LSB-first bitstream.

    16 channels produce exactly 22 bytes (176 bits), no padding needed.
    """
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


def rc_frame(channels: Sequence[int]) -> bytes:
    """Build a complete RC channels frame.

    Layout: [0xC8][len][0x16][22 payload bytes][crc] -- 26 bytes for 16
    channels, where `len` counts the body plus the CRC byte.
    """
    payload = pack_channels(channels)
    body = bytes([TYPE_RC_CHANNELS]) + payload
    return bytes([ADDR_MODULE, len(body) + 1]) + body + bytes([crc8(body)])


def parse_frames(buf: bytearray) -> Iterator[tuple[int, bytes]]:
    """Yield (frame_type, payload) from `buf`, consuming what it parses.

    Mutates `buf` in place: complete frames are removed, and a partial
    frame at the tail is left for the next call. On a bad length byte it
    pops a single byte to resync. Frames failing CRC are dropped.
    """
    while len(buf) >= 4:
        length = buf[1]
        if length < MIN_FRAME_LEN or length > MAX_FRAME_LEN:
            buf.pop(0)
            continue

        total = length + 2
        if len(buf) < total:
            # Might be a real frame still arriving -- but only if the
            # address byte is plausible. Otherwise we would stall here
            # forever waiting on a length we invented from noise.
            if buf[0] == ADDR_MODULE:
                return
            buf.pop(0)
            continue

        frame = bytes(buf[:total])
        body = frame[2:-1]
        if crc8(body) == frame[-1]:
            del buf[:total]
            yield body[0], body[1:]
        else:
            # A bad CRC means the length byte cannot be trusted either,
            # so advance one byte rather than consuming `total` and
            # dragging the parser out of alignment.
            buf.pop(0)
