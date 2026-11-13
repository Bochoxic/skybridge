"""Run a controller over a recorded flight, to see what it would have commanded.

**This is open loop.** The drone in the recording flew on the pilot's sticks, so
the controller's output never changes the height it sees next. It shows how a
controller reacts to real height, noise, latency and track dropouts. It cannot
show whether the loop is stable, or what the drone would have done.

The controller is called directly, at the recorded RC rate. The supervisor's
switch and stick-override rules are left out, because the recorded stick moves
all the time and those rules would stop the controller at once.
"""

from __future__ import annotations

import re
from collections import Counter
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..config import ModeConfig
from ..perception.types import DroneEstimate
from ..state import TelemetryState, Timestamped
from ..units import throttle_fraction, tick_to_deflection, us_to_tick
from .base import ControlContext, Controller, Disengage, Sticks

SCALARS = "Scalars:scalars"
NAMES = "SeriesLines:names"
TIMELINE = "monotonic"
DEFAULT_LATENCY = 0.04  # seconds from capture to the estimate reaching the RC loop


@dataclass(frozen=True, eq=False)
class FlightLog:
    """What a ``data/live/*_flight.rrd`` holds. All times are monotonic seconds."""

    rc_t: np.ndarray
    rc_us: Mapping[str, np.ndarray]     # channel name -> microseconds
    est_t: np.ndarray                   # capture time of each tracked frame
    pos: np.ndarray                     # Nx3, metres
    vel: np.ndarray                     # Nx3, m/s
    pos_std: np.ndarray                 # Nx3, metres
    since_update: np.ndarray            # seconds


@dataclass(frozen=True, eq=False)
class ReplayResult:
    t: np.ndarray                       # seconds from the start of the log, airborne only
    z: np.ndarray                       # latest height the controller could see (NaN: none)
    pilot: np.ndarray                   # recorded throttle, 0..1
    output: np.ndarray                  # controller throttle, 0..1 (NaN: it had dropped out)
    setpoint: float
    trim: float
    debug: dict[str, np.ndarray]
    dropouts: Counter = field(default_factory=Counter)  # reason -> ticks


@dataclass(frozen=True)
class ReplayStats:
    ticks: int
    running: float                      # fraction of ticks with an output
    stopped: int                        # ticks without one, whatever the cause
    mean_diff: float                    # controller - pilot
    rms_diff: float
    max_abs_diff: float
    correlation: float
    output_range: tuple[float, float]
    pilot_range: tuple[float, float]
    at_limit: float                     # fraction of outputs pinned at their extremes


def load_flight_log(path: str | Path) -> FlightLog:
    """Read a flight log written by ``rc_fly.py --perception --record``."""
    import rerun.chunk as rrc

    names: dict[str, list[str]] = {}
    times: dict[str, list[np.ndarray]] = {}
    values: dict[str, list[list[list[float]]]] = {}
    for chunk in rrc.RrdReader(str(path)).stream():
        entity = chunk.entity_path
        if not entity.startswith("/plots/"):
            continue
        batch = chunk.to_record_batch()
        if chunk.is_static:
            if NAMES in batch.schema.names:
                names[entity] = batch.column(NAMES)[0].as_py()
            continue
        if SCALARS not in batch.schema.names:
            continue
        times.setdefault(entity, []).append(
            batch.column(TIMELINE).cast("int64").to_numpy() / 1e9
        )
        values.setdefault(entity, []).append(batch.column(SCALARS).to_pylist())

    def series(entity: str) -> tuple[np.ndarray, np.ndarray]:
        if entity not in times:
            raise ValueError(f"{path}: no {entity} (was it recorded with --record?)")
        t = np.concatenate(times[entity])
        data = np.array([row for chunk_rows in values[entity] for row in chunk_rows], dtype=float)
        order = np.argsort(t, kind="stable")
        return t[order], data[order]

    rc_t, rc = series("/plots/rc")
    est_t, pos = series("/plots/position")
    _, vel = series("/plots/velocity")
    _, std = series("/plots/position_std")
    _, since = series("/plots/since_update")
    if "/plots/rc" not in names:
        raise ValueError(f"{path}: the RC channels are not named")
    rc_us = {name: rc[:, i] for i, name in enumerate(names["/plots/rc"])}
    return FlightLog(rc_t, rc_us, est_t, pos, vel, std, since[:, 0])


def airborne_mask(log: FlightLog, arm: ModeConfig, min_throttle: float) -> np.ndarray:
    """RC ticks that were armed with the throttle above `min_throttle`."""
    armed = np.array([arm.is_active(us) for us in log.rc_us["arm"]], dtype=bool)
    return armed & (_throttle(log) >= min_throttle)


def _throttle(log: FlightLog) -> np.ndarray:
    return np.array([throttle_fraction(us_to_tick(us)) for us in log.rc_us["throttle"]])


def _heights_at(log: FlightLog, ticks: np.ndarray, latency: float) -> np.ndarray:
    """Index of the newest estimate that had arrived by each tick (-1: none yet)."""
    return np.searchsorted(log.est_t + latency, ticks, side="right") - 1


def _estimate(log: FlightLog, i: int) -> DroneEstimate:
    """Rebuild one estimate. The log has no status, but since_update implies it."""
    since = float(log.since_update[i])
    return DroneEstimate(
        t=float(log.est_t[i]),
        seq=i,
        status="detected" if since == 0.0 else "coasting",
        valid=True,
        pos=_vec(log.pos[i]),
        vel=_vec(log.vel[i]),
        pos_std=_vec(log.pos_std[i]),
        since_update=since,
    )


def _vec(row: np.ndarray) -> tuple[float, float, float]:
    return float(row[0]), float(row[1]), float(row[2])


def run_replay(
    log: FlightLog,
    controller: Controller,
    arm: ModeConfig,
    *,
    setpoint: float | None = None,
    trim: float | None = None,
    latency: float = DEFAULT_LATENCY,
    min_throttle: float = 0.15,
) -> ReplayResult:
    """Run `controller` over the airborne part of `log`.

    `setpoint` and `trim` default to the median airborne height and throttle.
    The controller is engaged on a reference state holding those values, so it
    starts as if the pilot had switched it on there. After a drop-out it
    re-engages on the next tick, unlike in flight, so one dropout doesn't hide
    the rest of the log. Its integrator restarts each time.
    """
    mask = airborne_mask(log, arm, min_throttle)
    ticks = log.rc_t[mask]
    if ticks.size == 0:
        raise ValueError("no airborne ticks: armed with throttle above the minimum")
    pilot = _throttle(log)[mask]
    index = _heights_at(log, ticks, latency)
    seen = index >= 0
    z = np.full(ticks.size, np.nan)
    z[seen] = log.pos[index[seen], 2]
    if not seen.any():
        raise ValueError("no tracked height during the airborne part of this log")

    target = float(np.median(z[seen])) if setpoint is None else setpoint
    baseline = float(np.median(pilot)) if trim is None else trim
    output = np.full(ticks.size, np.nan)
    debug: dict[str, np.ndarray] = {}
    dropouts: Counter = Counter()
    deflection = {
        axis: np.array([tick_to_deflection(us_to_tick(us)) for us in log.rc_us[axis][mask]])
        for axis in ("roll", "pitch", "yaw")
    }

    engaged = False
    telemetry = TelemetryState()
    for k, now in enumerate(ticks):
        now = float(now)
        if not engaged:
            controller.engage(_reference(now, target, baseline, telemetry))
            engaged = True
        latest = None
        if index[k] >= 0:
            estimate = _estimate(log, int(index[k]))
            latest = Timestamped(estimate, estimate.t).at(now)
        sticks = Sticks(float(deflection["roll"][k]), float(deflection["pitch"][k]),
                        float(deflection["yaw"][k]), float(pilot[k]))
        dt = 0.0 if k == 0 else now - float(ticks[k - 1])
        try:
            result = controller.update(ControlContext(now, dt, sticks, latest, telemetry))
        except Disengage as exc:
            dropouts[_reason(str(exc))] += 1
            engaged = False
            continue
        if result.throttle is not None:
            output[k] = result.throttle
        for key, value in result.debug.items():
            debug.setdefault(key, np.full(ticks.size, np.nan))[k] = value

    return ReplayResult(ticks - log.rc_t[0], z, pilot, output, target, baseline, debug, dropouts)


def _reference(now: float, height: float, throttle: float, telemetry) -> ControlContext:
    """A steady state at the target: what the controller engages on."""
    estimate = DroneEstimate(t=now, seq=-1, status="detected", valid=True,
                             pos=(0.0, 0.0, height), vel=(0.0, 0.0, 0.0),
                             pos_std=(0.0, 0.0, 0.0), since_update=0.0)
    return ControlContext(now, 0.0, Sticks(throttle=throttle),
                          Timestamped(estimate, now).at(now), telemetry)


def _reason(message: str) -> str:
    """A drop-out reason without its numbers, so that equal causes count together."""
    return re.sub(r"[\s(]*[-+]?\d.*$", "", message)


def summarize(result: ReplayResult) -> ReplayStats:
    running = np.isfinite(result.output)
    if not running.any():
        raise ValueError(f"the controller never ran: {dict(result.dropouts)}")
    output, pilot = result.output[running], result.pilot[running]
    diff = output - pilot
    pinned = np.isclose(output, output.max(), atol=1e-6) | np.isclose(
        output, output.min(), atol=1e-6
    )
    constant = output.std() == 0.0 or pilot.std() == 0.0
    return ReplayStats(
        ticks=result.t.size,
        running=float(running.mean()),
        stopped=sum(result.dropouts.values()),
        mean_diff=float(diff.mean()),
        rms_diff=float(np.sqrt(np.mean(diff**2))),
        max_abs_diff=float(np.abs(diff).max()),
        correlation=0.0 if constant else float(np.corrcoef(output, pilot)[0, 1]),
        output_range=(float(output.min()), float(output.max())),
        pilot_range=(float(pilot.min()), float(pilot.max())),
        at_limit=float(pinned.mean()),
    )
