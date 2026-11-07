"""Joystick axis -> CRSF tick mapping, and the rc: config section."""

import pytest

from skybridge import ConfigError, DroneConfig
from skybridge.crsf.constants import CH_MAX, CH_MID, CH_MIN
from skybridge.rc import RcMapping, active_modes, axis_to_tick
from skybridge.units import tick_to_us

MODES = {
    "arm": {"aux": 1, "range": [1700, 2100]},
    "angle": {"aux": 2, "range": [1000, 1300]},
    "acro": {"aux": 2, "range": [1400, 2000]},
}


def test_axis_endpoints_hit_crsf_endpoints():
    assert axis_to_tick(-32767) == CH_MIN
    assert axis_to_tick(0) == CH_MID
    assert axis_to_tick(32767) == CH_MAX


def test_axis_endpoints_decode_like_edgetx():
    # EdgeTX +/-100% is 988..2012us, which is what the radio sends itself.
    assert tick_to_us(axis_to_tick(-32767)) == pytest.approx(988, abs=0.5)
    assert tick_to_us(axis_to_tick(32767)) == pytest.approx(2012, abs=0.5)


def test_axis_clamps_out_of_range():
    assert axis_to_tick(-32768) == CH_MIN


def test_default_mapping_is_identity_for_seven_axes():
    cfg = DroneConfig.from_dict({"modes": MODES})
    ticks = RcMapping.from_config(cfg).apply([32767, 0, -32767, 0, 32767, -32767, 0])
    assert ticks[:7] == [CH_MAX, CH_MID, CH_MIN, CH_MID, CH_MAX, CH_MIN, CH_MID]
    assert ticks[7:] == [CH_MIN] * 9


def test_unmapped_sticks_are_centred_and_aux_low():
    cfg = DroneConfig.from_dict({"rc": {"mapping": {"ch3": 0}}})
    ticks = RcMapping.from_config(cfg).apply([32767])
    assert ticks[0] == ticks[1] == ticks[3] == CH_MID   # roll, pitch, yaw
    assert ticks[2] == CH_MAX                            # throttle from axis 0
    assert ticks[4:] == [CH_MIN] * 12


def test_invert_and_remap():
    cfg = DroneConfig.from_dict(
        {"rc": {"mapping": {"ch1": {"axis": 1, "invert": True}, "ch2": 0}}}
    )
    ticks = RcMapping.from_config(cfg).apply([32767, 32767])
    assert ticks[0] == CH_MIN
    assert ticks[1] == CH_MAX


def test_mapping_beyond_device_axes_raises():
    cfg = DroneConfig.from_dict({"rc": {"mapping": {"ch1": 9}}})
    with pytest.raises(ValueError, match="axis 9"):
        RcMapping.from_config(cfg).apply([0] * 7)


def test_active_modes():
    cfg = DroneConfig.from_dict({"modes": MODES})
    mapping = RcMapping.from_config(cfg)
    ticks = mapping.apply([0, 0, -32767, 0, 32767, -27000, 0])   # CH6 ~1078us
    assert sorted(active_modes(cfg, ticks)) == ["angle", "arm"]


def test_full_low_switch_misses_a_range_starting_at_1000():
    # EdgeTX -100% is 988us: below a [1000, 1300] range. Betaflight would
    # not activate it either -- the switch endpoint or the range must move.
    cfg = DroneConfig.from_dict({"modes": MODES})
    ticks = RcMapping.from_config(cfg).apply([0, 0, -32767, 0, -32767, -32767, 0])
    assert active_modes(cfg, ticks) == []


@pytest.mark.parametrize(
    "rc, match",
    [
        ({"mapping": {"ch17": 0}}, "ch1..ch16"),
        ({"mapping": {"roll": 0}}, "ch1..ch16"),
        ({"mapping": {"ch1": {"invert": True}}}, "axis number"),
        ({"max_throttle": 0}, "max_throttle"),
        ({"bogus": 1}, "unknown rc keys"),
    ],
)
def test_rc_config_rejects_bad_input(rc, match):
    with pytest.raises(ConfigError, match=match):
        DroneConfig.from_dict({"rc": rc})


def test_arm_channel_must_be_mapped():
    with pytest.raises(ConfigError, match="arm channel"):
        DroneConfig.from_dict({"modes": MODES, "rc": {"mapping": {"ch1": 0}}})


def test_shipped_config_loads():
    cfg = DroneConfig.from_yaml("configs/drone.yaml")
    assert cfg.modes["arm"].channel in cfg.rc.mapping
