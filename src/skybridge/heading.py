"""The drone's heading in the camera frame: FC yaw, kept honest by the plate.

FC yaw (Betaflight's integrated gyro) follows every turn at once but walks
away from the truth at its gyro bias, around 0.5 deg/s on this drone. The
plate seen by the depth camera (:mod:`skybridge.perception.plate`) never
drifts, but is noisy and only known modulo 180. A two-state Kalman filter
combines them:

    heading = fc + offset       fc = -(FC yaw - FC yaw at arm)
    state     [offset, drift]   drift = how fast the offset grows, deg/s

Convention, the plate's: **0 = plate parallel to the image plane, + = left**
(counter-clockwise seen from above). FC yaw is right-positive, hence the
minus. The filter resets at every arm, with the heading at 0: arm with the
plate parallel to the camera, and if it isn't, the plate corrects the
offset within a second or two. FC yaw supplies the full circle, so the
plate only has to correct within +-90 degrees.

:class:`HeadingFilter` is clock-free and thread-free; :class:`HeadingTracker`
feeds it from the Bus and publishes ``heading/estimate``.
"""

from __future__ import annotations

import threading
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from .bus import Bus
from .config import ModeConfig
from .units import tick_to_us

TOPIC = "heading/estimate"


@dataclass(frozen=True)
class HeadingParams:
    plate_std: float = 8.0          # deg, one plate frame at full confidence
    full_conf_elong: float = 50.0   # plate elongation that counts as full confidence
    min_weight: float = 0.05        # a frame's weight never drops below this
    offset_walk: float = 0.5        # deg^2/s: FC errors other than a steady drift
    drift_walk: float = 0.002       # (deg/s)^2/s: how fast the gyro bias itself changes
    gate: float = 3.0               # sigma; plate frames further off are ignored...
    gate_after: int = 30            # ...once this many have been used
    init_offset_std: float = 30.0   # deg: how far from parallel the drone may be armed
    init_drift_std: float = 1.0     # deg/s
    fc_latency: float = 0.05        # s from FC attitude to it reaching this process
    history: float = 2.0            # s of FC yaw kept, to match late plate frames


@dataclass(frozen=True)
class HeadingEstimate:
    t: float
    heading_deg: float | None       # fused; continuous, not wrapped. None until armed
    fc_deg: float | None            # -(FC yaw - FC yaw at arm)
    offset_deg: float = 0.0
    drift_dps: float = 0.0
    offset_std: float = 0.0
    plate_deg: float | None = None  # this update's plate frame, unwrapped next to heading
    plate_used: bool = False        # False: ignored as an outlier, or not armed
    used: int = 0
    rejected: int = 0


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def wrap90(a: float) -> float:
    return (a + 90.0) % 180.0 - 90.0


class HeadingFilter:
    def __init__(self, params: HeadingParams = HeadingParams()) -> None:
        self.p = params
        self._raw: float | None = None      # last FC yaw as received
        self._yaw_u: float | None = None    # FC yaw, unwrapped
        self._fc_hist: deque[tuple[float, float]] = deque()  # (valid at, unwrapped yaw)
        self.disarm()

    @property
    def armed(self) -> bool:
        return self._armed

    def arm(self) -> None:
        """Start a new estimate: heading 0 at the FC yaw of this moment."""
        self.disarm()
        self._armed = True
        self._ref = self._yaw_u   # None if no FC yaw yet: taken from the next one

    def disarm(self) -> None:
        self._armed = False
        self._ref: float | None = None
        self.x = np.zeros(2)
        self.P = np.diag([self.p.init_offset_std ** 2, self.p.init_drift_std ** 2])
        self._t_state: float | None = None
        self.used = self.rejected = 0

    def on_fc_yaw(self, t: float, yaw_deg: float) -> HeadingEstimate | None:
        """An FC attitude yaw received at `t`. Returns the new estimate once armed."""
        if self._yaw_u is None:
            self._yaw_u = yaw_deg
        else:
            self._yaw_u += wrap180(yaw_deg - self._raw)
        self._raw = yaw_deg
        self._fc_hist.append((t - self.p.fc_latency, self._yaw_u))
        while self._fc_hist and self._fc_hist[0][0] < t - self.p.history:
            self._fc_hist.popleft()
        if self._armed and self._ref is None:
            self._ref = self._yaw_u
        return self.estimate(t)

    def on_plate(self, t: float, plate_deg: float, elong: float) -> HeadingEstimate:
        """A plate frame captured at `t`. Used if armed and FC yaw covers `t`."""
        fc = self._fc_at(t)
        if fc is None:
            return HeadingEstimate(t, None, None, plate_deg=plate_deg)
        self._predict(t)
        pred = fc + self.x[0]
        nu = wrap90(plate_deg - pred)
        weight = min(max(elong / self.p.full_conf_elong, self.p.min_weight), 1.0)
        S = self.P[0, 0] + self.p.plate_std ** 2 / weight
        used = self.used < self.p.gate_after or nu * nu / S <= self.p.gate ** 2
        if used:
            K = self.P[:, 0] / S
            self.x = self.x + K * nu
            self.P = self.P - np.outer(K, self.P[0, :])
            self.used += 1
        else:
            self.rejected += 1
        return self._make(t, fc, plate_deg=pred + nu, plate_used=used)

    def estimate(self, t: float) -> HeadingEstimate | None:
        """The heading at `t`, from the newest FC yaw. None until armed with FC yaw."""
        if not self._armed or self._ref is None:
            return None
        return self._make(t, -(self._yaw_u - self._ref))

    def _make(self, t, fc, **plate) -> HeadingEstimate:
        ahead = 0.0 if self._t_state is None else max(t - self._t_state, 0.0)
        offset = self.x[0] + self.x[1] * ahead
        return HeadingEstimate(
            t=t, heading_deg=fc + offset, fc_deg=fc, offset_deg=float(offset),
            drift_dps=float(self.x[1]), offset_std=float(np.sqrt(self.P[0, 0])),
            used=self.used, rejected=self.rejected, **plate)

    def _fc_at(self, t: float) -> float | None:
        """FC heading (from arm) at capture time `t`, interpolated. None if not covered."""
        if not self._armed or self._ref is None or not self._fc_hist:
            return None
        ts = [h[0] for h in self._fc_hist]
        if t < ts[0]:
            return None
        yaw = float(np.interp(t, ts, [h[1] for h in self._fc_hist]))
        return -(yaw - self._ref)

    def _predict(self, t: float) -> None:
        if self._t_state is not None and t > self._t_state:
            dt = t - self._t_state
            F = np.array([[1.0, dt], [0.0, 1.0]])
            self.x = F @ self.x
            self.P = F @ self.P @ F.T + np.diag([self.p.offset_walk * dt,
                                                 self.p.drift_walk * dt])
        if self._t_state is None or t > self._t_state:
            self._t_state = t


class HeadingTracker:
    """Runs a :class:`HeadingFilter` on Bus messages and publishes ``heading/estimate``.

    Arm comes from ``rc/sent`` (what the FC was sent, after the guards), FC
    yaw from ``telemetry/attitude``, the plate from ``perception/estimate``.
    Those arrive on two threads, so the filter is behind a lock.
    """

    def __init__(self, bus: Bus, arm: ModeConfig,
                 params: HeadingParams = HeadingParams()) -> None:
        self._bus = bus
        self._arm = arm
        self.filter = HeadingFilter(params)
        self._lock = threading.Lock()
        self._unsubscribe: list[Callable[[], None]] = []
        self._latest: HeadingEstimate | None = None

    @property
    def latest(self) -> HeadingEstimate | None:
        return self._latest

    def start(self) -> None:
        self._unsubscribe = [
            self._bus.subscribe("rc/sent", self._on_rc),
            self._bus.subscribe("telemetry/attitude", self._on_attitude),
            self._bus.subscribe("perception/estimate", self._on_estimate),
        ]

    def close(self) -> None:
        for unsubscribe in self._unsubscribe:
            unsubscribe()
        self._unsubscribe = []

    def _on_rc(self, topic: str, ticks: Any, t: float) -> None:
        armed = self._arm.is_active(tick_to_us(ticks[self._arm.channel]))
        with self._lock:
            if armed and not self.filter.armed:
                self.filter.arm()
            elif not armed and self.filter.armed:
                self.filter.disarm()
                self._latest = None

    def _on_attitude(self, topic: str, values: Any, t: float) -> None:
        if "yaw_deg" not in values:
            return
        with self._lock:
            est = self.filter.on_fc_yaw(t, values["yaw_deg"])
        self._publish(est)

    def _on_estimate(self, topic: str, e: Any, t: float) -> None:
        if getattr(e, "plate_yaw", None) is None:
            return
        with self._lock:
            est = self.filter.on_plate(e.t, e.plate_yaw, e.plate_elong or 0.0)
        self._publish(est)

    def _publish(self, est: HeadingEstimate | None) -> None:
        if est is None:
            return
        if est.heading_deg is not None:
            self._latest = est
        self._bus.publish(TOPIC, est, est.t)
