"""Conversions between physical units and CRSF channel values.

Two things here are subtle enough to be worth stating up front, both
verified against Betaflight `master` source rather than assumed.

**1. Tick <-> microseconds.**  Betaflight decodes CRSF ticks with an
affine map (`rx/crsf.c`)::

    us = 0.62477120195241 * tick + 881

so tick 172 is 988us, *not* 1000us -- the value the old reference script
assumed. Inverting that map, 1000/1500/2000us land on ticks 190/991/1791
(each within 0.5us), which is what :func:`us_to_tick` produces.

**2. Angle mode is not linear in stick deflection.**  `pidLevel()`
(`flight/pid.c`) computes::

    angleTarget = angle_limit * currentPidSetpoint / maxRcRate

where `currentPidSetpoint` comes from the *acro rate curve* and
`maxRcRate = applyRates(1.0)`. So::

    angle(d) = angle_limit * applyRates(d) / applyRates(1.0)

With stock ACTUAL rates (rc_rate=7, srate=67) asking for 20 degrees the
naive way commands 8.06 degrees.

The curve collapses to exactly linear when ``srate <= rc_rate``: in
`applyActualRates` the term ``stickMovement = max(0, srate*10 -
rc_rate*10)`` becomes zero, the expo term drops out entirely, and
``angleRate = d * rc_rate * 10``.  Setting ``rc_rate == srate`` in
Betaflight is therefore the clean fix, and `config.py` validates it.

For non-linear rates, :func:`deflection_for_angle` inverts the curve
numerically (bisection -- the curve is monotonic).
"""

from __future__ import annotations

from .crsf.constants import CH_MAX, CH_MIN, US_OFFSET, US_PER_TICK

# Nominal RC endpoints in microseconds. Ticks are derived from these via
# Betaflight's decode rather than hardcoded, so there is a single source
# of truth for the mapping.
US_MIN = 1000.0
US_MID = 1500.0
US_MAX = 2000.0


# ── Ticks and microseconds ────────────────────────────────────────────────────
def tick_to_us(tick: float) -> float:
    """Convert a CRSF tick to microseconds, as Betaflight decodes it."""
    return US_PER_TICK * tick + US_OFFSET


def us_to_tick(us: float) -> int:
    """Convert microseconds to the CRSF tick Betaflight will decode back.

    Inverse of :func:`tick_to_us`, rounded and clamped to the valid range.
    """
    tick = round((us - US_OFFSET) / US_PER_TICK)
    return clamp_tick(tick)


def clamp_tick(tick: int) -> int:
    return max(CH_MIN, min(CH_MAX, int(tick)))


# ── Deflection ────────────────────────────────────────────────────────────────
# "Deflection" is normalised stick position in [-1, 1], where 0 is centre.


def deflection_to_us(deflection: float) -> float:
    """Map deflection in [-1, 1] onto microseconds, centred on 1500us."""
    deflection = max(-1.0, min(1.0, deflection))
    span = (US_MAX - US_MID) if deflection >= 0 else (US_MID - US_MIN)
    return US_MID + deflection * span


def deflection_to_tick(deflection: float) -> int:
    """Map deflection in [-1, 1] onto a CRSF tick."""
    return us_to_tick(deflection_to_us(deflection))


def tick_to_deflection(tick: int) -> float:
    us = tick_to_us(tick)
    span = (US_MAX - US_MID) if us >= US_MID else (US_MID - US_MIN)
    return (us - US_MID) / span


# ── Betaflight rate curve ─────────────────────────────────────────────────────
def apply_actual_rates(
    deflection: float,
    rc_rate: float,
    srate: float,
    expo: float,
) -> float:
    """Betaflight's ACTUAL rates curve, returning deg/s.

    Direct transcription of `applyActualRates()` in `fc/rc.c`.
    """
    abs_d = abs(deflection)
    expof = expo / 100.0
    expof = abs_d * (deflection**5 * expof + deflection * (1 - expof))

    center_sensitivity = rc_rate * 10.0
    stick_movement = max(0.0, srate * 10.0 - center_sensitivity)
    return deflection * center_sensitivity + stick_movement * expof


def max_rate(rc_rate: float, srate: float, expo: float) -> float:
    """Maximum commandable rate in deg/s -- i.e. applyRates(1.0)."""
    return apply_actual_rates(1.0, rc_rate, srate, expo)


def is_linear(rc_rate: float, srate: float) -> bool:
    """True when the rate curve is exactly linear in deflection.

    Holds when ``srate <= rc_rate``, which zeroes the `stickMovement`
    term and removes the expo contribution entirely.
    """
    return srate * 10.0 <= rc_rate * 10.0


def deflection_for_rate(
    rate_dps: float,
    rc_rate: float,
    srate: float,
    expo: float,
) -> float:
    """Deflection needed to command `rate_dps` degrees/second.

    Inverts the rate curve by bisection. The curve is monotonic in
    deflection, so this always converges.
    """
    limit = max_rate(rc_rate, srate, expo)
    if limit <= 0:
        return 0.0

    sign = 1.0 if rate_dps >= 0 else -1.0
    target = min(abs(rate_dps), limit)

    if is_linear(rc_rate, srate):
        return sign * (target / limit)

    lo, hi = 0.0, 1.0
    for _ in range(64):
        mid = (lo + hi) / 2
        if apply_actual_rates(mid, rc_rate, srate, expo) < target:
            lo = mid
        else:
            hi = mid
    return sign * (lo + hi) / 2


def angle_for_deflection(
    deflection: float,
    angle_limit: float,
    rc_rate: float,
    srate: float,
    expo: float,
) -> float:
    """Angle in degrees that Betaflight will command for a deflection.

    ``angle = angle_limit * applyRates(d) / applyRates(1.0)``
    """
    limit = max_rate(rc_rate, srate, expo)
    if limit <= 0:
        return 0.0
    return angle_limit * apply_actual_rates(deflection, rc_rate, srate, expo) / limit


def deflection_for_angle(
    angle_deg: float,
    angle_limit: float,
    rc_rate: float,
    srate: float,
    expo: float,
) -> float:
    """Deflection needed to command `angle_deg` degrees in Angle mode.

    Because angle routes through the rate curve, this is the rate
    inversion scaled by the angle limit -- not simply angle/angle_limit.
    """
    if angle_limit <= 0:
        return 0.0
    sign = 1.0 if angle_deg >= 0 else -1.0
    target = min(abs(angle_deg), angle_limit)

    # angle/angle_limit is the same normalised fraction as rate/max_rate.
    limit = max_rate(rc_rate, srate, expo)
    return deflection_for_rate(
        sign * (target / angle_limit) * limit, rc_rate, srate, expo
    )
