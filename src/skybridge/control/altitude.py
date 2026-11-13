"""Altitude hold: throttle from perception height, with a PID.

The target is the height when the controller engages, and the trim is the
throttle at that moment, so taking over causes no jump. The pilot was holding
roughly hover to get there, which is a better hover estimate than a constant
because it follows the battery.

The D term uses the tracker's filtered climb rate instead of differentiating
height, so depth noise is not amplified into the motors.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any

from ..config import ConfigError
from .base import ControlContext, ControlOutput, Disengage

MAX_DT = 0.1  # seconds; a stalled loop must not dump into the integrator


@dataclass(frozen=True)
class AltitudeHoldParams:
    kp: float = 0.08
    ki: float = 0.02
    kd: float = 0.08
    authority: float = 0.10
    i_limit: float = 0.08
    min_engage_throttle: float = 0.15
    max_engage_climb: float = 0.3
    max_age: float = 0.2
    max_since_update: float = 0.2
    max_error: float = 0.5

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> AltitudeHoldParams:
        known = {f.name for f in fields(cls)}
        if unknown := set(raw) - known:
            raise ConfigError(f"altitude_hold: unknown params {sorted(unknown)}")
        params = cls(**raw)
        for name in ("kp", "ki", "kd", "i_limit", "min_engage_throttle"):
            if getattr(params, name) < 0:
                raise ConfigError(f"altitude_hold: {name} must be >= 0")
        for name in ("max_engage_climb", "max_age", "max_since_update", "max_error"):
            if getattr(params, name) <= 0:
                raise ConfigError(f"altitude_hold: {name} must be > 0")
        if not 0.0 < params.authority <= 0.5:
            raise ConfigError("altitude_hold: authority must be in (0, 0.5]")
        if params.i_limit > params.authority:
            raise ConfigError("altitude_hold: i_limit must not exceed authority")
        return params


class AltitudeHold:
    name = "altitude_hold"
    axes = frozenset({"throttle"})

    def __init__(self, params: AltitudeHoldParams | None = None) -> None:
        self.params = params or AltitudeHoldParams()
        self._setpoint = 0.0
        self._trim = 0.0
        self._integral = 0.0

    @classmethod
    def from_params(cls, raw: dict[str, Any]) -> AltitudeHold:
        return cls(AltitudeHoldParams.from_dict(raw))

    def engage(self, ctx: ControlContext) -> None:
        p = self.params
        z, vz = self._height(ctx)
        status = ctx.estimate.value.status  # type: ignore[union-attr]  # _height checked it
        if status != "detected":
            raise Disengage(f"track is {status}")
        if ctx.sticks.throttle < p.min_engage_throttle:
            raise Disengage("throttle too low")
        if abs(vz) > p.max_engage_climb:
            raise Disengage(f"moving too fast ({vz:+.1f} m/s)")
        self._setpoint = z
        self._trim = ctx.sticks.throttle
        self._integral = 0.0

    def update(self, ctx: ControlContext) -> ControlOutput:
        p = self.params
        z, vz = self._height(ctx)
        error = self._setpoint - z
        if abs(error) > p.max_error:
            raise Disengage(f"height error {error:+.2f} m")

        dt = max(0.0, min(ctx.dt, MAX_DT))
        proportional = p.kp * error
        damping = -p.kd * vz

        # Anti-windup: don't integrate further into a limit already reached.
        delta = proportional + self._integral + damping
        if not (abs(delta) >= p.authority and error * delta > 0):
            self._integral = _clamp(self._integral + p.ki * error * dt, p.i_limit)

        delta = _clamp(proportional + self._integral + damping, p.authority)
        throttle = self._trim + delta
        return ControlOutput(
            throttle=throttle,
            text=f"ALT {z:.2f}>{self._setpoint:.2f}m T{throttle * 100:.0f}%",
            debug={
                "setpoint": self._setpoint,
                "z": z,
                "vz": vz,
                "error": error,
                "integral": self._integral,
                "throttle": throttle,
            },
        )

    def _height(self, ctx: ControlContext) -> tuple[float, float]:
        """Height projected to now and climb rate, or Disengage if not trustworthy."""
        p = self.params
        latest = ctx.estimate
        if latest is None:
            raise Disengage("no track")
        estimate = latest.value
        # `valid` stays true through up to max_coast (1.5 s) of pure prediction,
        # so it is not enough on its own: since_update is what bounds that.
        if not estimate.valid or estimate.vel is None:
            raise Disengage(f"track {estimate.status}")
        if latest.is_stale(p.max_age):
            raise Disengage(f"estimate stale ({latest.age:.2f} s)")
        if estimate.since_update is None or estimate.since_update > p.max_since_update:
            raise Disengage("track coasting")
        position = estimate.at(ctx.t)
        if position is None:
            raise Disengage(f"track {estimate.status}")
        return position[2], estimate.vel[2]


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
