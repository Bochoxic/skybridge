"""Config parsing and validation tests."""

import warnings

import pytest

from skybridge import ConfigError, DroneConfig


def base(**overrides) -> dict:
    cfg = {
        "betaflight": {
            "angle_limit": 60,
            "rates": {"type": "actual", "rc_rate": 67, "srate": 67, "expo": 0},
        },
        "modes": {"arm": {"aux": 1, "range": [1700, 2100]}},
        "safety": {"max_angle": 30},
    }
    cfg.update(overrides)
    return cfg


def test_defaults_are_usable():
    cfg = DroneConfig.from_dict(base())

    assert cfg.serial.baud == 115200
    assert cfg.serial.rc_hz == 50
    assert cfg.channels.roll == 0
    assert cfg.betaflight.angle_limit == 60


def test_scalar_rates_apply_to_all_axes():
    cfg = DroneConfig.from_dict(base())

    assert cfg.betaflight.roll.rc_rate == 67
    assert cfg.betaflight.yaw.srate == 67


def test_per_axis_rates():
    cfg = DroneConfig.from_dict(
        base(
            betaflight={
                "angle_limit": 60,
                "rates": {
                    "type": "actual",
                    "rc_rate": {"roll": 67, "pitch": 50, "yaw": 40},
                    "srate": {"roll": 67, "pitch": 50, "yaw": 40},
                },
            }
        )
    )

    assert cfg.betaflight.roll.rc_rate == 67
    assert cfg.betaflight.pitch.rc_rate == 50
    assert cfg.betaflight.yaw.rc_rate == 40


def test_nonlinear_rates_warn():
    """Stock BF rates are non-linear; the user should be told."""
    with pytest.warns(UserWarning, match="non-linear"):
        DroneConfig.from_dict(
            base(
                betaflight={
                    "angle_limit": 60,
                    "rates": {"type": "actual", "rc_rate": 7, "srate": 67},
                }
            )
        )


def test_linear_rates_do_not_warn():
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        DroneConfig.from_dict(base())


def test_angle_limit_outside_betaflight_range_rejected():
    for bad in (5, 100):
        with pytest.raises(ConfigError, match="angle_limit"):
            DroneConfig.from_dict(
                base(
                    betaflight={
                        "angle_limit": bad,
                        "rates": {"type": "actual", "rc_rate": 67, "srate": 67},
                    }
                )
            )


def test_safety_max_angle_above_angle_limit_rejected():
    with pytest.raises(ConfigError, match="max_angle"):
        DroneConfig.from_dict(base(safety={"max_angle": 80}))


def test_unsupported_rates_type_rejected():
    with pytest.raises(ConfigError, match="rates_type"):
        DroneConfig.from_dict(
            base(
                betaflight={
                    "angle_limit": 60,
                    "rates": {"type": "quick", "rc_rate": 67, "srate": 67},
                }
            )
        )


def test_bad_mode_range_rejected():
    with pytest.raises(ConfigError, match="range"):
        DroneConfig.from_dict(
            base(modes={"arm": {"aux": 1, "range": [2100, 1700]}})
        )


def test_malformed_mode_rejected():
    with pytest.raises(ConfigError, match="aux"):
        DroneConfig.from_dict(base(modes={"arm": {"range": [1700, 2100]}}))


def test_unknown_mode_lookup_lists_known_modes():
    cfg = DroneConfig.from_dict(base())

    with pytest.raises(ConfigError, match="known modes: arm"):
        cfg.mode("nope")


def test_mode_targets_centre_of_range():
    """BF activation is half-open, so aim for the middle, not an edge."""
    cfg = DroneConfig.from_dict(base())
    assert cfg.mode("arm").value_us == 1900


def test_unknown_betaflight_key_rejected():
    with pytest.raises(ConfigError, match="unknown betaflight keys"):
        DroneConfig.from_dict(
            base(
                betaflight={
                    "angle_limit": 60,
                    "rates": {"type": "actual", "rc_rate": 67, "srate": 67},
                    "typo_here": 1,
                }
            )
        )


def test_shipped_config_is_valid(tmp_path):
    """configs/drone.yaml must actually load."""
    from pathlib import Path

    path = Path(__file__).parent.parent / "configs" / "drone.yaml"
    cfg = DroneConfig.from_yaml(path)

    assert cfg.betaflight.roll.is_linear
    assert "arm" in cfg.modes


# ── control ───────────────────────────────────────────────────────────────────
ALT = {
    "switch": {"aux": 3, "range": [1700, 2100]},
    "axes": ["throttle"],
    "require_mode": "angle",
}
MODES = {"arm": {"aux": 1, "range": [1700, 2100]}, "angle": {"aux": 2, "range": [900, 1300]}}
EIGHT_CHANNELS = {"mapping": {f"ch{i + 1}": {"axis": i} for i in range(8)}}


def control(**controllers) -> dict:
    return base(modes=MODES, rc=EIGHT_CHANNELS, control={"controllers": controllers})


def test_control_is_optional():
    assert DroneConfig.from_dict(base()).control.controllers == {}


def test_controller_parses_with_its_own_switch_and_axes():
    cfg = DroneConfig.from_dict(control(alt={**ALT, "params": {"kp": 0.1}}))
    alt = cfg.control.controllers["alt"]
    assert alt.switch.channel == 6
    assert alt.axes == {"throttle"}
    assert alt.require_mode == "angle"
    assert alt.stick_override == 0.15
    assert alt.params == {"kp": 0.1}


def test_each_controller_has_its_own_settings():
    other = {"switch": {"aux": 4, "range": [1700, 2100]}, "axes": ["roll", "pitch"],
             "require_mode": "angle", "stick_override": 0.3}
    cfg = DroneConfig.from_dict(control(alt=ALT, pos=other))
    assert cfg.control.controllers["alt"].axes == {"throttle"}
    assert cfg.control.controllers["pos"].axes == {"roll", "pitch"}
    assert cfg.control.controllers["pos"].stick_override == 0.3


def test_same_switch_disjoint_ranges_may_share_an_axis():
    other = {**ALT, "switch": {"aux": 3, "range": [1400, 1600]}}
    assert len(DroneConfig.from_dict(control(a=ALT, b=other)).control.controllers) == 2


def test_touching_ranges_do_not_overlap():
    other = {**ALT, "switch": {"aux": 3, "range": [1400, 1700]}}
    DroneConfig.from_dict(control(a=ALT, b=other))  # half-open: 1700 belongs to `a` only


@pytest.mark.parametrize("other_range", [[1700, 2100], [1600, 1701], [1800, 1900]])
def test_overlapping_switch_ranges_are_rejected(other_range):
    other = {**ALT, "switch": {"aux": 3, "range": other_range}}
    with pytest.raises(ConfigError, match="overlap"):
        DroneConfig.from_dict(control(a=ALT, b=other))


def test_axis_shared_across_different_switches_is_rejected():
    other = {**ALT, "switch": {"aux": 4, "range": [1700, 2100]}}
    with pytest.raises(ConfigError, match="throttle"):
        DroneConfig.from_dict(control(a=ALT, b=other))


def test_different_switches_with_disjoint_axes_are_allowed():
    other = {**ALT, "switch": {"aux": 4, "range": [1700, 2100]}, "axes": ["roll"]}
    assert len(DroneConfig.from_dict(control(a=ALT, b=other)).control.controllers) == 2


@pytest.mark.parametrize("change, match", [
    ({"axes": ["arm"]}, "unknown axes"),
    ({"axes": []}, "must not be empty"),
    ({"axes": "throttle"}, "must be a list"),
    ({"switch": {"aux": 2, "range": [1700, 2100]}}, "already used by mode"),
    ({"switch": {"aux": 1, "range": [1700, 2100]}}, "already used by mode"),
    ({"switch": {"aux": 9, "range": [1700, 2100]}}, "not in rc.mapping"),
    ({"switch": {"aux": 3, "range": [2000, 1000]}}, "must satisfy"),
    ({"switch": {"aux": 3}}, "expected"),
    ({"require_mode": "acro"}, "unknown require_mode"),
    ({"stick_override": 0.0}, "stick_override"),
    ({"typo": 1}, "unknown keys"),
])
def test_bad_controller_entries_are_rejected(change, match):
    with pytest.raises(ConfigError, match=match):
        DroneConfig.from_dict(control(a={**ALT, **change}))


@pytest.mark.parametrize("key", ["switch", "axes", "require_mode"])
def test_controller_keys_are_required(key):
    entry = {k: v for k, v in ALT.items() if k != key}
    with pytest.raises(ConfigError, match="missing"):
        DroneConfig.from_dict(control(a=entry))


def test_unknown_control_key_is_rejected():
    with pytest.raises(ConfigError, match="unknown control keys"):
        DroneConfig.from_dict(base(control={"controller": {}}))
