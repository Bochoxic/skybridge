"""RC passthrough guards, end to end through a Bridge and a fake transport."""

import pytest

from skybridge import Bridge, Bus, DroneConfig, LoopbackTransport, RcPassthrough
from skybridge.crsf import parse_frames
from skybridge.crsf.constants import CH_MAX, CH_MIN, TYPE_RC_CHANNELS
from skybridge.passthrough import ARM_THROTTLE, ARM_WAIT_OFF, cap_throttle
from skybridge.rc import FakeRcSource
from skybridge.units import tick_to_us

LOW, MID, HIGH = -32767, 0, 32767
ARM_AXIS, THR_AXIS = 4, 2

CONFIG = {
    "serial": {"port": "fake", "rc_hz": 50},
    "modes": {
        "arm": {"aux": 1, "range": [1700, 2100]},
        "angle": {"aux": 2, "range": [1000, 1300]},
    },
    "safety": {"command_timeout": 0.2},
}


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def sent_channels(transport):
    buf = bytearray(transport.written)
    frames = [p for t, p in parse_frames(buf) if t == TYPE_RC_CHANNELS]
    bits = int.from_bytes(frames[-1], "little")
    return [(bits >> (11 * i)) & 0x7FF for i in range(16)]


def axes(arm=LOW, throttle=LOW, roll=MID):
    a = [roll, MID, throttle, MID, arm, LOW, MID]
    return tuple(a)


@pytest.fixture
def rig():
    cfg = DroneConfig.from_dict(CONFIG)
    clock, transport, source = FakeClock(), LoopbackTransport(), FakeRcSource()
    bridge = Bridge(cfg, transport, clock=clock)
    return RcPassthrough(bridge, source, cfg), source, transport, clock


def armed_on_wire(transport):
    return tick_to_us(sent_channels(transport)[4]) >= 1700


def test_waits_for_radio_state(rig):
    pt, source, _, _ = rig
    assert pt.step().state == "waiting"


def test_channels_pass_through_verbatim(rig):
    pt, source, transport, _ = rig
    source.set(axes(roll=HIGH))
    status = pt.step()
    pt.bridge.tick()

    assert status.state == "live"
    ch = sent_channels(transport)
    assert ch[0] == CH_MAX and ch[2] == CH_MIN
    assert list(status.ticks) == ch


def test_arm_switch_on_at_start_is_held_off_until_cycled(rig):
    pt, source, transport, _ = rig
    source.set(axes(arm=HIGH))
    status = pt.step()
    pt.bridge.tick()
    assert not status.armed and status.note == ARM_WAIT_OFF
    assert not armed_on_wire(transport)

    source.set(axes(arm=LOW))
    pt.step()
    source.set(axes(arm=HIGH))
    status = pt.step()
    pt.bridge.tick()
    assert status.armed and armed_on_wire(transport)


def test_arming_with_throttle_up_is_refused_until_cycled(rig):
    pt, source, transport, _ = rig
    source.set(axes(arm=LOW, throttle=MID))
    pt.step()

    source.set(axes(arm=HIGH, throttle=MID))
    status = pt.step()
    assert not status.armed and status.note == ARM_THROTTLE

    # Lowering throttle alone is not enough: the switch must be cycled.
    source.set(axes(arm=HIGH, throttle=LOW))
    assert not pt.step().armed

    source.set(axes(arm=LOW, throttle=LOW))
    pt.step()
    source.set(axes(arm=HIGH, throttle=LOW))
    assert pt.step().armed


def test_throttle_may_rise_once_armed(rig):
    pt, source, transport, _ = rig
    for a in (axes(arm=LOW), axes(arm=HIGH), axes(arm=HIGH, throttle=HIGH)):
        source.set(a)
        status = pt.step()
    pt.bridge.tick()
    assert status.armed and armed_on_wire(transport)
    assert sent_channels(transport)[2] == CH_MAX


def test_throttle_cap_scales_channel():
    ticks = cap_throttle([0, 0, CH_MAX] + [0] * 13, channel=2, max_throttle=0.5)
    assert ticks[2] == round(CH_MIN + (CH_MAX - CH_MIN) * 0.5)
    assert cap_throttle([0, 0, CH_MIN] + [0] * 13, 2, 0.5)[2] == CH_MIN


def test_max_throttle_override(rig):
    pt, source, transport, _ = rig
    pt2 = RcPassthrough(pt.bridge, source, pt.config, max_throttle=0.4)
    source.set(axes(throttle=HIGH))
    pt2.step()
    pt.bridge.tick()
    assert tick_to_us(sent_channels(transport)[2]) == pytest.approx(988 + 0.4 * 1024, abs=1)


def test_dead_radio_lets_watchdog_disarm(rig):
    pt, source, transport, clock = rig
    for a in (axes(arm=LOW), axes(arm=HIGH)):
        source.set(a)
        pt.step()
    pt.bridge.tick()
    assert armed_on_wire(transport)

    source.alive = False
    assert pt.step().state == "dead"
    clock.now += 0.1
    pt.bridge.tick()
    assert armed_on_wire(transport)          # still inside command_timeout

    clock.now += 0.15
    pt.bridge.tick()
    assert not armed_on_wire(transport)
    assert pt.bridge.failsafe_active


def test_bridge_publishes_sent_frames_and_telemetry():
    cfg = DroneConfig.from_dict(CONFIG)
    bus, seen = Bus(), []
    bus.subscribe("", lambda topic, payload, t: seen.append(topic))
    transport = LoopbackTransport()
    bridge = Bridge(cfg, transport, clock=FakeClock(), bus=bus)

    # A FLIGHT_MODE frame: [addr][len][type][payload][crc]
    from skybridge.crsf.protocol import crc8
    body = bytes([0x21]) + b"ANGL\x00"
    transport.feed(bytes([0xC8, len(body) + 1]) + body + bytes([crc8(body)]))
    bridge.tick()

    assert seen == ["rc/sent", "telemetry/flight_mode"]


def test_physical_setter_takes_over_from_raw(rig):
    pt, source, transport, _ = rig
    source.set(axes(roll=HIGH))
    pt.step()
    pt.bridge.set_throttle(0.0)
    pt.bridge.tick()
    assert tick_to_us(sent_channels(transport)[0]) == pytest.approx(1500, abs=1)


def test_set_channels_validates_length(rig):
    pt, *_ = rig
    with pytest.raises(ValueError):
        pt.bridge.set_channels([CH_MIN] * 8)
