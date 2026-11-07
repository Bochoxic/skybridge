"""Unit conversion tests.

The reference numbers here come from Betaflight `master` source:
`rx/crsf.c` for the tick map, `fc/rc.c` (applyActualRates) and
`flight/pid.c` (pidLevel) for the rate and angle curves.
"""

import pytest

from skybridge import units


# ── Ticks and microseconds ────────────────────────────────────────────────────
def test_betaflight_decode_anchors():
    """The anchors Betaflight documents in rx/crsf.c.

    BF's comment rounds these to whole microseconds (988/1500/2012); the
    exact affine map lands within a microsecond of each.
    """
    assert units.tick_to_us(172) == pytest.approx(988, abs=1)
    assert units.tick_to_us(992) == pytest.approx(1500, abs=1)
    assert units.tick_to_us(1811) == pytest.approx(2012, abs=1)


def test_round_microseconds_map_within_half_a_microsecond():
    """The old script's 172 <-> 1000us assumption was ~12us out."""
    for us in (1000, 1500, 2000):
        assert units.tick_to_us(units.us_to_tick(us)) == pytest.approx(us, abs=0.5)


def test_us_tick_roundtrip_is_stable():
    for tick in range(200, 1800, 37):
        assert units.us_to_tick(units.tick_to_us(tick)) == tick


def test_ticks_are_clamped_to_valid_range():
    assert units.clamp_tick(-99) == 172
    assert units.clamp_tick(99999) == 1811
    assert units.us_to_tick(500) == 172
    assert units.us_to_tick(5000) == 1811


# ── Deflection ────────────────────────────────────────────────────────────────
def test_deflection_endpoints():
    assert units.deflection_to_us(0.0) == pytest.approx(1500)
    assert units.deflection_to_us(1.0) == pytest.approx(2000)
    assert units.deflection_to_us(-1.0) == pytest.approx(1000)


def test_deflection_roundtrip():
    for d in (-1.0, -0.5, 0.0, 0.25, 1.0):
        tick = units.deflection_to_tick(d)
        assert units.tick_to_deflection(tick) == pytest.approx(d, abs=0.01)


def test_deflection_is_clamped():
    assert units.deflection_to_us(5.0) == pytest.approx(2000)
    assert units.deflection_to_us(-5.0) == pytest.approx(1000)


# ── Betaflight rate curve ─────────────────────────────────────────────────────
def test_stock_rates_are_not_linear():
    """rc_rate=7, srate=67 is the Betaflight default and is non-linear."""
    assert not units.is_linear(rc_rate=7, srate=67)


def test_equal_rates_are_linear():
    """srate <= rc_rate zeroes stickMovement, collapsing the curve."""
    assert units.is_linear(rc_rate=67, srate=67)
    assert units.is_linear(rc_rate=67, srate=20)


def test_max_rate_matches_betaflight():
    assert units.max_rate(7, 67, 0) == pytest.approx(670.0)
    assert units.max_rate(67, 67, 0) == pytest.approx(670.0)
    # Linear at 7/7 would cripple acro -- the reason config uses 67.
    assert units.max_rate(7, 7, 0) == pytest.approx(70.0)


def test_expo_is_irrelevant_when_linear():
    """With stickMovement == 0 the expo term drops out entirely."""
    for expo in (0, 20, 100):
        assert units.apply_actual_rates(0.5, 67, 67, expo) == pytest.approx(
            units.apply_actual_rates(0.5, 67, 67, 0)
        )


# ── Angle mapping ─────────────────────────────────────────────────────────────
def test_naive_angle_mapping_would_be_badly_wrong():
    """The finding that motivates this whole module.

    With stock rates, treating deflection as angle/angle_limit commands
    8 degrees when you asked for 20.
    """
    actual = units.angle_for_deflection(20 / 60, 60, 7, 67, 0)
    assert actual == pytest.approx(8.06, abs=0.05)


def test_angle_inversion_is_exact_with_stock_rates():
    d = units.deflection_for_angle(20, 60, 7, 67, 0)
    assert d == pytest.approx(0.5545, abs=0.001)
    assert units.angle_for_deflection(d, 60, 7, 67, 0) == pytest.approx(20, abs=0.01)


def test_angle_is_linear_with_equal_rates():
    for want in (5, 10, 20, 30, 45, 60):
        d = units.deflection_for_angle(want, 60, 67, 67, 0)
        assert d == pytest.approx(want / 60, abs=1e-6)
        assert units.angle_for_deflection(d, 60, 67, 67, 0) == pytest.approx(
            want, abs=0.01
        )


def test_angle_request_beyond_limit_is_clamped():
    d = units.deflection_for_angle(999, 60, 67, 67, 0)
    assert d == pytest.approx(1.0)


def test_negative_angles_are_symmetric():
    pos = units.deflection_for_angle(20, 60, 7, 67, 0)
    neg = units.deflection_for_angle(-20, 60, 7, 67, 0)
    assert neg == pytest.approx(-pos)


# ── Rate mapping ──────────────────────────────────────────────────────────────
def test_rate_inversion_roundtrips():
    for want in (0, 50, 200, 500, 670):
        d = units.deflection_for_rate(want, 7, 67, 0)
        assert units.apply_actual_rates(d, 7, 67, 0) == pytest.approx(want, abs=0.5)


def test_rate_beyond_max_is_clamped():
    assert units.deflection_for_rate(99999, 67, 67, 0) == pytest.approx(1.0)
