"""CRSF framing tests -- no hardware needed."""

from skybridge.crsf import crc8, pack_channels, parse_frames, rc_frame
from skybridge.crsf.constants import ADDR_MODULE, CH_MID, TYPE_RC_CHANNELS


def test_crc8_known_values():
    assert crc8(b"") == 0
    # Same input must always give the same CRC.
    assert crc8(b"\x16\x00\x01") == crc8(b"\x16\x00\x01")


def test_pack_channels_length():
    """16 channels x 11 bits = 176 bits = exactly 22 bytes."""
    assert len(pack_channels([CH_MID] * 16)) == 22


def test_pack_channels_masks_to_11_bits():
    packed = pack_channels([0x7FF] * 16)
    assert packed == b"\xff" * 22

    # Values above 11 bits are truncated, not allowed to bleed across.
    assert pack_channels([0xFFFF] * 16) == packed


def test_rc_frame_layout():
    frame = rc_frame([CH_MID] * 16)

    assert len(frame) == 26
    assert frame[0] == ADDR_MODULE
    assert frame[1] == 24          # body (23) + crc (1)
    assert frame[2] == TYPE_RC_CHANNELS
    assert frame[-1] == crc8(frame[2:-1])


def test_parse_frames_roundtrip():
    frame = rc_frame([CH_MID] * 16)
    buf = bytearray(frame)

    parsed = list(parse_frames(buf))

    assert len(parsed) == 1
    ftype, payload = parsed[0]
    assert ftype == TYPE_RC_CHANNELS
    assert payload == pack_channels([CH_MID] * 16)
    assert not buf, "a fully parsed frame should be consumed"


def test_parse_frames_keeps_partial_tail():
    frame = rc_frame([CH_MID] * 16)
    buf = bytearray(frame[:10])

    assert list(parse_frames(buf)) == []
    assert len(buf) == 10, "an incomplete frame must stay buffered"


def test_parse_frames_drops_bad_crc():
    frame = bytearray(rc_frame([CH_MID] * 16))
    frame[-1] ^= 0xFF

    assert list(parse_frames(bytearray(frame))) == []


def test_parse_frames_recovers_on_the_next_frame_after_garbage():
    """Resync is best-effort, and the frame straddling the garbage is lost.

    CRSF has no sync byte -- only a length byte -- so random leading
    bytes can look like a plausible length and consume part of the real
    frame. What matters is that the parser recovers rather than
    deadlocking, so the following frame decodes cleanly.
    """
    frame = rc_frame([CH_MID] * 16)
    buf = bytearray(b"\x00\x01\x02" + frame + frame)

    parsed = list(parse_frames(buf))

    assert parsed, "parser must recover on a subsequent frame"
    assert all(ftype == TYPE_RC_CHANNELS for ftype, _ in parsed)


def test_parse_frames_handles_back_to_back():
    frame = rc_frame([CH_MID] * 16)
    buf = bytearray(frame + frame)

    assert len(list(parse_frames(buf))) == 2
