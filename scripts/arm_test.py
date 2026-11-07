#!/usr/bin/env python3
"""Arm briefly, hold, then disarm. A first-light hardware check.

Throttle stays at zero the whole time, so motors should not spin up --
but the drone IS armed. REMOVE THE PROPS.

Prints live telemetry throughout so you can confirm the link is healthy
while armed, and disarms on exit whatever happens (Ctrl-C, exception,
or the timer expiring).

Usage:
    python scripts/arm_test.py              # arm for 3 seconds
    python scripts/arm_test.py --seconds 5
    python scripts/arm_test.py --countdown 10
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from skybridge import Bridge, DroneConfig  # noqa: E402

CONFIG = Path(__file__).parent.parent / "configs" / "drone.yaml"


def status_line(bridge: Bridge, prefix: str) -> str:
    att = bridge.state.attitude
    if att is None:
        state = "no telemetry"
    elif att.is_stale(0.5):
        state = f"STALE {att.age:.1f}s"
    else:
        state = (
            f"roll={att.value.roll_deg:+6.1f} "
            f"pitch={att.value.pitch_deg:+6.1f} "
            f"yaw={att.value.yaw_deg:+7.1f}"
        )

    link = bridge.state.link
    lq = f" LQ={link.value.uplink_lq:3d}%" if link else ""
    return f"\r{prefix} | {state}{lq} | {bridge.state.attitude_hz:4.1f}Hz\033[K"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=3.0, help="time armed")
    ap.add_argument("--countdown", type=int, default=5, help="seconds before arming")
    ap.add_argument("--config", default=str(CONFIG))
    args = ap.parse_args()

    cfg = DroneConfig.from_yaml(args.config)

    print("=" * 60)
    print("  ARM TEST -- REMOVE THE PROPS BEFORE RUNNING THIS")
    print("=" * 60)
    print(f"  port          {cfg.serial.port} @ {cfg.serial.baud}")
    print(f"  armed for     {args.seconds:.0f}s")
    print(f"  throttle      0.0 (motors should not spin)")
    print(f"  watchdog      disarms after {cfg.safety.command_timeout:.1f}s idle")
    print("=" * 60)
    print()

    with Bridge(cfg) as bridge:
        bridge.set_mode("angle")
        bridge.set_throttle(0.0)

        # Let telemetry come up before committing to anything.
        for remaining in range(args.countdown, 0, -1):
            print(
                status_line(bridge, f"arming in {remaining}s  (Ctrl-C aborts)"),
                end="",
                flush=True,
            )
            time.sleep(1.0)

        print("\n")
        if bridge.state.attitude is None:
            print("No telemetry -- is the drone powered and linked?")
            print("Aborting rather than arming blind.")
            return

        bridge.arm()
        print(">>> ARMED <<<\n")

        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            left = deadline - time.monotonic()
            # Keep commanding so the watchdog sees us as alive.
            bridge.set_angle(roll=0.0, pitch=0.0, yaw_rate=0.0)
            print(status_line(bridge, f"ARMED {left:4.1f}s left"), end="", flush=True)
            time.sleep(0.02)

        print("\n")
        bridge.disarm()
        print(">>> DISARMED <<<")
        time.sleep(0.3)

    print("\nBridge closed.")


if __name__ == "__main__":
    main()
