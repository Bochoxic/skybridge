#!/usr/bin/env python3
"""Run controllers on a recorded flight and compare them with the pilot's throttle.

Open loop: the recorded drone flew on the sticks, so this shows how a controller
reacts to real height data, not whether it would have held the drone. Every
configured controller is replayed by default, on the same data, side by side
(e.g. the PID altitude_hold and the trained altitude_policy). The target
is the median airborne height and the trim the median throttle, unless given.

Works on the flight logs from ``rc_fly.py --perception --record``.

Usage:
    python scripts/control_replay.py                    # newest log, opens Rerun
    python scripts/control_replay.py data/live/<run>_flight.rrd
    python scripts/control_replay.py --setpoint 0.4 --set kp=0.12 --set kd=0.1
    python scripts/control_replay.py --controller altitude_hold --controller altitude_policy
    python scripts/control_replay.py --set altitude_policy.on_timeout=pilot
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


def parse_value(text: str) -> float | str:
    try:
        return float(text)
    except ValueError:
        return text


def parse_overrides(pairs: list[str], names: list[str]):
    """``controller.param=value`` goes to that controller; a plain ``param=value`` to every
    replayed controller that has the param (see build_controllers)."""
    targeted: dict[str, dict[str, float | str]] = {name: {} for name in names}
    shared: dict[str, float | str] = {}
    for pair in pairs:
        key, sep, value = pair.partition("=")
        if not sep or not key:
            sys.exit(f"--set {pair!r}: expected name=value or controller.name=value")
        target, dot, param = key.rpartition(".")
        if not dot:
            shared[param] = parse_value(value)
        elif target in targeted:
            targeted[target][param] = parse_value(value)
        else:
            sys.exit(f"--set {pair!r}: {target!r} is not one of the replayed controllers")
    return targeted, shared


def build_controllers(entries, names, targeted, shared) -> dict:
    """Each controller with its overrides. A shared param a controller doesn't have is
    left out for that one; it is an error only if no controller has it."""
    controllers, used = {}, set()
    for name in names:
        extra = dict(shared)
        while True:
            try:
                controllers[name] = CONTROLLERS[name]({**entries[name].params, **extra,
                                                       **targeted[name]})
                break
            except ConfigError as exc:
                unknown = {k for k in extra if repr(k) in str(exc) and "unknown" in str(exc)}
                if not unknown:
                    sys.exit(f"{name}: {exc}")
                for k in unknown:
                    del extra[k]
        used |= set(extra)
    if missing := set(shared) - used:
        sys.exit(f"--set: no replayed controller has {sorted(missing)}")
    return controllers


def plot(results: dict[str, ReplayResult], name: str, save: str | None) -> None:
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

    first = next(iter(results.values()))

    def series(path: str, names: list[str], columns: list) -> None:
        rr.log(path, rr.SeriesLines(names=names), static=True, recording=rec)
        for i, t in enumerate(first.t):
            rr.set_time("flight_time", duration=float(t), recording=rec)
            rr.log(path, rr.Scalars([float(c[i]) for c in columns]), recording=rec)

    target = [first.setpoint] * first.t.size
    series("height", ["z", "target"], [first.z, target])
    series("throttle", ["pilot", *results], [first.pilot, *(r.output for r in results.values())])
    series("difference", [f"{n} - pilot" for n in results],
           [r.output - r.pilot for r in results.values()])
    rec.flush()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="flight log (default: the newest in data/live)")
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--controller", action="append",
                    help="controller to replay (repeatable; default: every one under "
                         "control.controllers, side by side)")
    ap.add_argument("--setpoint", type=float, help="target height, m (default: median)")
    ap.add_argument("--trim", type=float, help="baseline throttle, 0..1 (default: median)")
    ap.add_argument("--latency", type=float, default=DEFAULT_LATENCY,
                    help="seconds from capture to the estimate reaching the loop")
    ap.add_argument("--set", action="append", default=[], metavar="[CONTROLLER.]NAME=VALUE",
                    help="override a param, e.g. --set kp=0.12 or "
                         "--set altitude_policy.on_timeout=pilot (repeatable)")
    ap.add_argument("--no-viewer", action="store_true", help="don't open Rerun")
    ap.add_argument("--save", help="write the plots to this .rrd instead of opening Rerun")
    args = ap.parse_args()

    cfg = DroneConfig.from_yaml(args.config)
    entries = cfg.control.controllers
    names = args.controller or [n for n in entries if n in CONTROLLERS]
    for name in names:
        if name not in entries or name not in CONTROLLERS:
            sys.exit(f"unknown controller {name!r}; configured: {', '.join(entries) or '(none)'}")
    if not names:
        sys.exit("no controllers configured under control.controllers")
    targeted, shared = parse_overrides(args.set, names)
    controllers = build_controllers(entries, names, targeted, shared)

    path = Path(args.log) if args.log else newest_log()
    log = load_flight_log(path)
    results: dict[str, ReplayResult] = {}
    stats = {}
    for name, controller in controllers.items():
        try:
            results[name] = run_replay(log, controller, cfg.modes["arm"], setpoint=args.setpoint,
                                       trim=args.trim, latency=args.latency)
            stats[name] = summarize(results[name])
        except ValueError as exc:
            sys.exit(f"{name}: {exc}")

    first = next(iter(results.values()))
    seen = first.z[~np.isnan(first.z)]
    n_ticks = next(iter(stats.values())).ticks
    print(f"log        {path.name}  (open loop: no output flew the drone)")
    print(f"airborne   {n_ticks / cfg.serial.rc_hz:.1f} s, "
          f"height {seen.min():.2f}..{seen.max():.2f} m")
    print(f"target     {first.setpoint:.3f} m   trim {first.trim:.3f}   "
          f"pilot {next(iter(stats.values())).pilot_range[0]:.3f}.."
          f"{next(iter(stats.values())).pilot_range[1]:.3f}")
    width = max(len(n) for n in names)
    print(f"\n{'':{width}}                                  controller - pilot")
    print(f"{'controller':{width}}  running  output range   at limit   mean    rms    max   corr")
    for name, st in stats.items():
        print(f"{name:{width}}  {st.running:6.1%}  {st.output_range[0]:.3f}..{st.output_range[1]:.3f}"
              f"  {st.at_limit:7.1%}  {st.mean_diff:+.3f}  {st.rms_diff:.3f}  "
              f"{st.max_abs_diff:.3f}  {st.correlation:+.2f}")
    for name, result in results.items():
        if result.dropouts:
            print(f"\n{name} stopped for:")
            for reason, count in result.dropouts.most_common():
                print(f"  {count:5d}  {reason}")
        timed_out = result.debug.get("timed_out")
        if timed_out is not None and np.nanmax(timed_out) > 0:
            print(f"\n{name}: the camera timed out and the throttle latched at 0 from "
                  f"{result.t[int(np.nanargmax(timed_out))]:.1f} s on. To see the rest, "
                  f"replay with --set {name}.on_timeout=pilot")
    if len(results) > 1:
        names_ = list(results)
        a, b = results[names_[0]].output, results[names_[1]].output
        both = np.isfinite(a) & np.isfinite(b)
        if both.any():
            d = b[both] - a[both]
            print(f"\n{names_[1]} - {names_[0]}: mean {d.mean():+.3f}  "
                  f"rms {np.sqrt(np.mean(d ** 2)):.3f}  max {np.abs(d).max():.3f}")

    if args.save or not args.no_viewer:
        plot(results, f"{path.stem}_{'_'.join(names)}", args.save)


if __name__ == "__main__":
    main()
