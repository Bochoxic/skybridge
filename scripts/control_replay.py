#!/usr/bin/env python3
"""Run a controller on a recorded flight and compare it with the pilot's throttle.

Open loop: the recorded drone flew on the sticks, so this shows how a controller
reacts to real height data, not whether it would have held the drone. The target
is the median airborne height and the trim the median throttle, unless given.

Works on the flight logs from ``rc_fly.py --perception --record``.

Usage:
    python scripts/control_replay.py                    # newest log, opens Rerun
    python scripts/control_replay.py data/live/<run>_flight.rrd
    python scripts/control_replay.py --setpoint 0.4 --set kp=0.12 --set kd=0.1
    python scripts/control_replay.py --no-viewer        # numbers only
    python scripts/control_replay.py --save out.rrd     # write the plots, open no viewer
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from skybridge import ConfigError, DroneConfig  # noqa: E402
from skybridge.control import CONTROLLERS  # noqa: E402
from skybridge.control.replay import (  # noqa: E402
    DEFAULT_LATENCY,
    ReplayResult,
    load_flight_log,
    run_replay,
    summarize,
)

ROOT = Path(__file__).parent.parent
CONFIG = ROOT / "configs" / "drone.yaml"
LOG_DIR = ROOT / "data" / "live"


def newest_log() -> Path:
    logs = sorted(LOG_DIR.glob("*_flight.rrd"))
    if not logs:
        sys.exit(f"no *_flight.rrd in {LOG_DIR}: fly with --perception --record first")
    return logs[-1]


def parse_overrides(pairs: list[str]) -> dict[str, float]:
    overrides = {}
    for pair in pairs:
        key, sep, value = pair.partition("=")
        try:
            if not sep:
                raise ValueError
            overrides[key] = float(value)
        except ValueError:
            sys.exit(f"--set {pair!r}: expected name=number")
    return overrides


def plot(result: ReplayResult, name: str, save: str | None) -> None:
    import rerun as rr
    import rerun.blueprint as rrb

    blueprint = rrb.Blueprint(rrb.Vertical(
        rrb.TimeSeriesView(name="height (m)", origin="height"),
        rrb.TimeSeriesView(name="throttle (0..1)", origin="throttle"),
        rrb.TimeSeriesView(name="controller - pilot", origin="difference"),
    ))
    rec = rr.RecordingStream("skybridge_control_replay", recording_id=name)
    if save:
        rec.save(save, default_blueprint=blueprint)
    else:
        rec.spawn(default_blueprint=blueprint)

    def series(path: str, names: list[str], columns: list) -> None:
        rr.log(path, rr.SeriesLines(names=names), static=True, recording=rec)
        for i, t in enumerate(result.t):
            rr.set_time("flight_time", duration=float(t), recording=rec)
            rr.log(path, rr.Scalars([float(c[i]) for c in columns]), recording=rec)

    target = [result.setpoint] * result.t.size
    series("height", ["z", "target"], [result.z, target])
    series("throttle", ["pilot", "controller"], [result.pilot, result.output])
    series("difference", ["controller - pilot"], [result.output - result.pilot])
    rec.flush()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="flight log (default: the newest in data/live)")
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--controller", help="default: the first under control.controllers")
    ap.add_argument("--setpoint", type=float, help="target height, m (default: median)")
    ap.add_argument("--trim", type=float, help="baseline throttle, 0..1 (default: median)")
    ap.add_argument("--latency", type=float, default=DEFAULT_LATENCY,
                    help="seconds from capture to the estimate reaching the loop")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a controller param, e.g. --set kp=0.12 (repeatable)")
    ap.add_argument("--no-viewer", action="store_true", help="don't open Rerun")
    ap.add_argument("--save", help="write the plots to this .rrd instead of opening Rerun")
    args = ap.parse_args()

    cfg = DroneConfig.from_yaml(args.config)
    entries = cfg.control.controllers
    name = args.controller or next(iter(entries), None)
    if name not in entries or name not in CONTROLLERS:
        sys.exit(f"unknown controller {name!r}; configured: {', '.join(entries) or '(none)'}")
    try:
        controller = CONTROLLERS[name]({**entries[name].params, **parse_overrides(args.set)})
    except ConfigError as exc:
        sys.exit(str(exc))

    path = Path(args.log) if args.log else newest_log()
    log = load_flight_log(path)
    try:
        result = run_replay(log, controller, cfg.modes["arm"], setpoint=args.setpoint,
                            trim=args.trim, latency=args.latency)
        stats = summarize(result)
    except ValueError as exc:
        sys.exit(str(exc))

    seen = result.z[~np.isnan(result.z)]
    print(f"log        {path.name}")
    print(f"controller {name}  (open loop: the output did not fly the drone)")
    print(f"airborne   {stats.ticks / cfg.serial.rc_hz:.1f} s, "
          f"height {seen.min():.2f}..{seen.max():.2f} m")
    print(f"target     {result.setpoint:.3f} m   trim {result.trim:.3f}")
    print(f"running    {stats.running:.1%} of airborne ticks; stopped for {stats.stopped} ticks:")
    for reason, count in result.dropouts.most_common():
        print(f"             {count:4d}  {reason}")
    print(f"pilot      {stats.pilot_range[0]:.3f}..{stats.pilot_range[1]:.3f}")
    print(f"controller {stats.output_range[0]:.3f}..{stats.output_range[1]:.3f}"
          f"   at its limits {stats.at_limit:.1%} of the time")
    print(f"controller - pilot: mean {stats.mean_diff:+.3f}  rms {stats.rms_diff:.3f}  "
          f"max {stats.max_abs_diff:.3f}  correlation {stats.correlation:+.2f}")

    if args.save or not args.no_viewer:
        plot(result, f"{path.stem}_{name}", args.save)


if __name__ == "__main__":
    main()
