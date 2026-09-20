"""Telemetry state with per-field staleness.

Staleness is tracked per field rather than globally: ATTITUDE arrives at
roughly 20Hz, BATTERY at ~1Hz and FLIGHT_MODE only when it changes, so a
single "is the telemetry stale" flag would mark battery permanently
stale. Each value instead carries the instant it was received.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace
from typing import Generic, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class Timestamped(Generic[T]):
    value: T
    received: float
    _now: float | None = None

    @property
    def age(self) -> float:
        """Seconds since this value arrived."""
        now = self._now if self._now is not None else time.monotonic()
        return now - self.received

    def is_stale(self, max_age: float) -> bool:
        return self.age > max_age

    def at(self, now: float) -> Timestamped[T]:
        """Pin this value's age to a fixed instant, for coherent snapshots."""
        return replace(self, _now=now)


@dataclass(frozen=True)
class Attitude:
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class Battery:
    voltage_V: float = 0.0
    current_A: float = 0.0
    capacity_mAh: int = 0
    remaining_pct: int = 0


@dataclass(frozen=True)
class LinkStats:
    uplink_rssi_dbm: int = 0
    uplink_lq: int = 0
    uplink_snr: int = 0
    downlink_lq: int = 0
    rf_mode: int = 0


@dataclass(frozen=True)
class Altitude:
    alt_m: float = 0.0
    vario_ms: float = 0.0


@dataclass(frozen=True)
class TelemetryState:
    """An immutable snapshot of the most recent telemetry.

    All ages within one snapshot are measured against the same instant,
    so comparisons between fields are coherent.
    """

    attitude: Timestamped[Attitude] | None = None
    battery: Timestamped[Battery] | None = None
    link: Timestamped[LinkStats] | None = None
    altitude: Timestamped[Altitude] | None = None
    flight_mode: Timestamped[str] | None = None
    attitude_hz: float = 0.0

    def at(self, now: float) -> TelemetryState:
        """Return this snapshot with every age pinned to `now`."""
        return TelemetryState(
            attitude=self.attitude.at(now) if self.attitude else None,
            battery=self.battery.at(now) if self.battery else None,
            link=self.link.at(now) if self.link else None,
            altitude=self.altitude.at(now) if self.altitude else None,
            flight_mode=self.flight_mode.at(now) if self.flight_mode else None,
            attitude_hz=self.attitude_hz,
        )
