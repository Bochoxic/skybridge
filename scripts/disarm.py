#!/usr/bin/env python3
"""Disarm the drone and hold a safe RC output.

Run this when something went wrong: a control script crashed, the drone
is armed and you want it stopped now.

Sends ARM low and throttle minimum for a couple of seconds so the flight
controller definitely sees it, then exits.

Usage:
    python scripts/disarm.py
    python scripts/disarm.py --hold 10     # keep holding for 10s
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from skybridge import Bridge, DroneConfig  # noqa: E402

CONFIG = Path(__file__).parent.parent / "configs" / "drone.yaml"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", type=float, default=2.0, help="seconds to hold")
    ap.add_argument("--config", default=str(CONFIG))
    args = ap.parse_args()

    cfg = DroneConfig.from_yaml(args.config)

    with Bridge(cfg) as bridge:
        bridge.disarm()
        bridge.set_throttle(0.0)

        print(f"Holding DISARM + throttle min on {cfg.serial.port} "
              f"for {args.hold:.0f}s...")

        deadline = time.monotonic() + args.hold
        try:
            while time.monotonic() < deadline:
                # Keep refreshing so the watchdog never trips into
                # failsafe -- we want a deliberate disarm, held steady.
                bridge.disarm()
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass

    print("Disarmed. Port closed.")


if __name__ == "__main__":
    main()
