#!/usr/bin/env python3
"""Example: a closed-loop controller using the Bridge.

Demonstrates the intended shape of control code -- the bridge keeps RC
flowing at 50Hz on its own thread while this loop runs at whatever rate
it likes.

THIS ARMS THE DRONE. Remove the props before running it.

Usage:
    python scripts/example_control_loop.py [--arm]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from skybridge import Bridge, DroneConfig  # noqa: E402

CONFIG = Path(__file__).parent.parent / "configs" / "drone.yaml"


def telemetry_status(bridge: Bridge) -> str:
    """One-line summary of the current telemetry."""
    att = bridge.state.attitude
    if att is None:
        state = "no telemetry yet"
    elif att.is_stale(0.5):
        state = f"STALE ({att.age:.1f}s)"
    else:
        state = (
            f"roll={att.value.roll_deg:+6.1f} "
            f"pitch={att.value.pitch_deg:+6.1f} "
            f"yaw={att.value.yaw_deg:+7.1f}"
        )

    link = bridge.state.link
    lq = f" LQ={link.value.uplink_lq:3d}%" if link else ""
    return f"{state}{lq} | {bridge.state.attitude_hz:4.1f}Hz"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", action="store_true", help="actually arm (props off!)")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--countdown", type=int, default=5, help="seconds before arming")
    args = ap.parse_args()

    cfg = DroneConfig.from_yaml(CONFIG)
    print(f"Loaded {CONFIG}")
    print(f"  angle_limit  {cfg.betaflight.angle_limit} deg")
    print(f"  max_angle    {cfg.safety.max_angle} deg (library clamp)")
    print(f"  rates linear {cfg.betaflight.roll.is_linear}")
    print()

    with Bridge(cfg) as bridge:
        bridge.set_mode("angle")
        bridge.set_throttle(0.0)

        if args.arm:
            # Count down while still commanding: a bare sleep here would
            # let the stale-command watchdog trip, and arm() would fire
            # into a failsafe that cancels it straight away.
            for remaining in range(args.countdown, 0, -1):
                bridge.set_angle(roll=0.0, pitch=0.0, yaw_rate=0.0)
                print(
                    f"\rarming in {remaining}s  (Ctrl-C aborts) | "
                    f"{telemetry_status(bridge)}\033[K",
                    end="",
                    flush=True,
                )
                time.sleep(1.0)

            print()
            if bridge.state.attitude is None:
                print("\nNo telemetry -- is the drone powered and linked?")
                print("Aborting rather than arming blind.")
                return

            bridge.arm()
            print(">>> ARMED <<<\n")
        else:
            print("Not arming (pass --arm). Sending sticks only.\n")

        start = time.monotonic()
        try:
            while (t := time.monotonic() - start) < args.seconds:
                # A gentle roll oscillation, +/-1 degree at 0.2Hz.
                roll = 2.0 * math.sin(2 * math.pi * 0.5 * t)
                bridge.set_angle(roll=roll, pitch=0.0, yaw_rate=0.0)

                print(
                    f"\rcmd_roll={roll:+6.1f}deg | {telemetry_status(bridge)}"
                    f"\033[K",
                    end="",
                    flush=True,
                )
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\nInterrupted.")

    # Leaving the `with` block disarms and closes the port.
    print("\nBridge closed (disarmed).")


if __name__ == "__main__":
    main()
