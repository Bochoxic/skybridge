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
