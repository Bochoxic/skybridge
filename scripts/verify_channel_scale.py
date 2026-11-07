#!/usr/bin/env python3
"""Verify the CRSF tick<->microsecond mapping against Betaflight.

Steps one channel through known microsecond values, holding each so you
can read it in the Betaflight Configurator's **Receiver tab** and confirm
the number matches what we claim to be sending.

Why this matters: the old reference script mapped 1000us to tick 172, but
Betaflight decodes tick 172 as 988us (see `rx/crsf.c`). This script sends
with both mappings so you can see the difference directly.

Usage:
    python scripts/verify_channel_scale.py [port] [baud]
    python scripts/verify_channel_scale.py --old      # old 172/1811 map
    python scripts/verify_channel_scale.py --channel 1
"""

from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent.parent / "src"))

import serial  # noqa: E402

from skybridge import units  # noqa: E402
from skybridge.crsf import rc_frame  # noqa: E402
from skybridge.crsf.constants import CH_MAX, CH_MIN, NUM_CHANNELS  # noqa: E402

STEPS_US = [1000, 1200, 1500, 1800, 2000]
HOLD_S = 4.0
RC_HZ = 50.0


def old_us_to_tick(us: float) -> int:
    """The reference script's mapping: 1000us -> 172, 2000us -> 1811."""
    us = max(1000.0, min(2000.0, us))
    return int(CH_MIN + (us - 1000) / 1000 * (CH_MAX - CH_MIN))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("baud", nargs="?", type=int, default=115200)
    ap.add_argument("--channel", type=int, default=0, help="0-indexed (0=roll)")
    ap.add_argument(
        "--old", action="store_true", help="use the old 172/1811 mapping"
    )
    args = ap.parse_args()

    convert = old_us_to_tick if args.old else units.us_to_tick
    label = "OLD (172/1811)" if args.old else "NEW (Betaflight-exact)"

    ser = serial.Serial(args.port, args.baud, timeout=0)

    print(f"Opened {args.port} @ {args.baud}")
    print(f"Mapping: {label}")
    print(f"Driving channel {args.channel + 1} (index {args.channel})")
    print()
    print("Open Betaflight Configurator -> Receiver tab and compare.")
    print("Throttle is held at minimum and no aux channels are raised.\n")
    print(f"{'sent us':>10}  {'tick':>6}  {'BF decodes':>12}  {'error':>8}")
    print("-" * 44)

    try:
        for want_us in STEPS_US:
            tick = convert(want_us)
            decoded = units.tick_to_us(tick)
            print(
                f"{want_us:>10}  {tick:>6}  {decoded:>11.1f}us  "
                f"{decoded - want_us:>+7.1f}",
                flush=True,
            )

            channels = [units.us_to_tick(1500)] * NUM_CHANNELS
            channels[2] = units.us_to_tick(1000)      # throttle low
            for idx in range(4, NUM_CHANNELS):        # aux channels low
                channels[idx] = CH_MIN
            channels[args.channel] = tick

            frame = rc_frame(channels)
            deadline = time.monotonic() + HOLD_S
            while time.monotonic() < deadline:
                ser.write(frame)
                time.sleep(1.0 / RC_HZ)

        print("\nDone. The Receiver tab should have shown each 'sent us' value.")
        if args.old:
            print("With --old you should see ~12us of error at the endpoints.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
