"""Bridge behaviour tests, driven by a fake transport and a fake clock.

The clock is injected so watchdog timing is deterministic rather than
depending on real sleeps.
"""

import pytest

from skybridge import (
    ArmRefused,
    Bridge,
    DroneConfig,
    LoopbackTransport,
    ModeMismatch,
)
from skybridge.crsf import parse_frames
from skybridge.crsf.constants import TYPE_RC_CHANNELS
from skybridge.units import tick_to_us


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt


CONFIG = {
    "serial": {"port": "fake", "baud": 115200, "rc_hz": 50},
    "betaflight": {
        "angle_limit": 60,
        "rates": {
            "type": "actual",
            "rc_rate": 67,
            "srate": 67,
            "expo": 0,
        },
    },
    "modes": {
        "arm": {"aux": 1, "range": [1700, 2100]},
        "angle": {"aux": 2, "range": [1300, 1700]},
        "acro": {"aux": 2, "range": [900, 1300]},
    },
    "safety": {"command_timeout": 0.2, "max_angle": 30, "max_rate": 200},
}


@pytest.fixture
def setup():
    cfg = DroneConfig.from_dict(CONFIG)
    clock = FakeClock()
    transport = LoopbackTransport()
    bridge = Bridge(cfg, transport, clock=clock)
    return bridge, transport, clock


def channels_of(transport) -> list[int]:
    """Decode the channel values from the last frame written."""
    buf = bytearray(transport.written)
    frames = [p for t, p in parse_frames(buf) if t == TYPE_RC_CHANNELS]
    assert frames, "no RC frame was written"

    payload = frames[-1]
    bits = int.from_bytes(payload, "little")
    return [(bits >> (11 * i)) & 0x7FF for i in range(16)]


# ── Framing ───────────────────────────────────────────────────────────────────
def test_tick_writes_a_valid_frame(setup):
    bridge, transport, _ = setup
    bridge.tick()

    channels = channels_of(transport)
    assert len(channels) == 16


def test_neutral_sticks_are_centred(setup):
    bridge, transport, _ = setup
    bridge.tick()

    ch = channels_of(transport)
    assert tick_to_us(ch[0]) == pytest.approx(1500, abs=1)
    assert tick_to_us(ch[1]) == pytest.approx(1500, abs=1)
    assert tick_to_us(ch[3]) == pytest.approx(1500, abs=1)


def test_throttle_maps_to_full_range(setup):
    bridge, transport, _ = setup

    bridge.set_throttle(0.0)
    bridge.tick()
    assert tick_to_us(channels_of(transport)[2]) == pytest.approx(1000, abs=1)

    bridge.set_throttle(1.0)
    bridge.tick()
    assert tick_to_us(channels_of(transport)[2]) == pytest.approx(2000, abs=1)


# ── Angle commands ────────────────────────────────────────────────────────────
def test_set_angle_produces_expected_channel(setup):
    bridge, transport, _ = setup
    bridge.set_mode("angle")
    bridge.set_angle(roll=30)      # half of the 60 degree limit
    bridge.tick()

    roll_us = tick_to_us(channels_of(transport)[0])
    assert roll_us == pytest.approx(1750, abs=2)


def test_set_angle_is_clamped_to_safety_limit(setup):
    bridge, transport, _ = setup
    bridge.set_mode("angle")

    bridge.set_angle(roll=999)
    bridge.tick()
    clamped = tick_to_us(channels_of(transport)[0])

    bridge.set_angle(roll=30)      # == safety.max_angle
    bridge.tick()
    assert clamped == pytest.approx(tick_to_us(channels_of(transport)[0]))


# ── Mode guarding ─────────────────────────────────────────────────────────────
def test_set_angle_in_acro_raises(setup):
    bridge, _, _ = setup
    bridge.set_mode("acro")

    with pytest.raises(ModeMismatch):
        bridge.set_angle(roll=10)


def test_set_rates_in_angle_raises(setup):
    bridge, _, _ = setup
    bridge.set_mode("angle")

    with pytest.raises(ModeMismatch):
        bridge.set_rates(roll=100)


def test_mode_sets_its_aux_channel(setup):
    bridge, transport, _ = setup
    bridge.set_mode("angle")
    bridge.tick()

    # angle -> aux2 -> channel index 5, centre of [1300, 1700]
    assert tick_to_us(channels_of(transport)[5]) == pytest.approx(1500, abs=2)


# ── Arming ────────────────────────────────────────────────────────────────────
def test_arm_refused_with_throttle_up(setup):
    bridge, _, _ = setup
    bridge.set_throttle(0.5)

    with pytest.raises(ArmRefused):
        bridge.arm()


def test_arm_allowed_at_zero_throttle(setup):
    bridge, transport, _ = setup
    bridge.set_throttle(0.0)
    bridge.arm()
    bridge.tick()

    # arm -> aux1 -> channel index 4, centre of [1700, 2100]
    assert tick_to_us(channels_of(transport)[4]) == pytest.approx(1900, abs=2)


def test_disarm_always_allowed(setup):
    bridge, transport, _ = setup
    bridge.arm()
    bridge.disarm()
    bridge.tick()

    assert tick_to_us(channels_of(transport)[4]) < 1100


# ── Watchdog ──────────────────────────────────────────────────────────────────
def test_watchdog_disarms_on_stale_command(setup):
    bridge, transport, clock = setup
    bridge.set_mode("angle")
    bridge.arm()
    bridge.set_throttle(0.5)
    bridge.tick()
    assert not bridge.failsafe_active

    clock.advance(0.5)             # beyond command_timeout
    bridge.tick()

    assert bridge.failsafe_active
    ch = channels_of(transport)
    assert tick_to_us(ch[4]) < 1100, "ARM should be cut"
    assert tick_to_us(ch[2]) == pytest.approx(1000, abs=1), "throttle to min"


def test_watchdog_does_not_fire_before_first_command(setup):
    bridge, _, clock = setup
    clock.advance(10.0)
    bridge.tick()

    assert not bridge.failsafe_active


def test_new_command_clears_failsafe(setup):
    bridge, _, clock = setup
    bridge.set_throttle(0.5)
    clock.advance(0.5)
    bridge.tick()
    assert bridge.failsafe_active

    bridge.set_throttle(0.4)
    assert not bridge.failsafe_active


# ── Telemetry ─────────────────────────────────────────────────────────────────
def test_attitude_telemetry_is_decoded(setup):
    import math
    import struct

    from skybridge.crsf.protocol import crc8

    bridge, transport, _ = setup

    # pitch, roll, yaw in raw/10000 radian units
    payload = struct.pack(
        ">hhh",
        int(math.radians(5) * 10000),
        int(math.radians(-10) * 10000),
        int(math.radians(90) * 10000),
    )
    body = bytes([0x1E]) + payload
    transport.feed(bytes([0xC8, len(body) + 1]) + body + bytes([crc8(body)]))

    bridge.tick()

    att = bridge.state.attitude
    assert att is not None
    assert att.value.pitch_deg == pytest.approx(5, abs=0.1)
    assert att.value.roll_deg == pytest.approx(-10, abs=0.1)
    assert att.value.yaw_deg == pytest.approx(90, abs=0.1)


def test_telemetry_age_tracks_the_clock(setup):
    import math
    import struct

    from skybridge.crsf.protocol import crc8

    bridge, transport, clock = setup

    payload = struct.pack(">hhh", 0, 0, 0)
    body = bytes([0x1E]) + payload
    transport.feed(bytes([0xC8, len(body) + 1]) + body + bytes([crc8(body)]))
    bridge.tick()

    clock.advance(2.0)
    assert bridge.state.attitude.age == pytest.approx(2.0)
    assert bridge.state.attitude.is_stale(0.5)
