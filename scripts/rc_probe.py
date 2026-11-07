#!/usr/bin/env python3
"""Show what the radio sends, channel by channel. No TX module, nothing armed.

Use this before rc_fly.py to check that every stick and switch lands on
the CRSF channel drone.yaml expects, and that the arm / mode switches
actually reach their Betaflight ranges. EdgeTX's -100% is 988us, so a
range that starts at 1000 will NOT activate on a fully-low switch.

Usage:
    python scripts/rc_probe.py
    python scripts/rc_probe.py --device /dev/input/js1
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from skybridge import DroneConfig  # noqa: E402
from skybridge.rc import JoystickSource, RcMapping, active_modes  # noqa: E402
from skybridge.units import tick_to_us  # noqa: E402

CONFIG = Path(__file__).parent.parent / "configs" / "drone.yaml"


def label_for(cfg: DroneConfig, ch: int) -> str:
    cmap = cfg.channels
    sticks = {cmap.roll: "roll", cmap.pitch: "pitch", cmap.throttle: "throttle", cmap.yaw: "yaw"}
    if ch in sticks:
        return sticks[ch]
    modes = [n for n, m in cfg.modes.items() if m.channel == ch]
    return f"AUX{ch - 3} " + "/".join(modes) if ch >= 4 else ""


def bar(us: float, width: int = 31) -> str:
    pos = int((min(max(us, 988), 2012) - 988) / 1024 * (width - 1))
    return "".join("|" if i == pos else ("+" if i == width // 2 else "-") for i in range(width))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--device", help="override rc.device")
    args = ap.parse_args()

    cfg = DroneConfig.from_yaml(args.config)
    mapping = RcMapping.from_config(cfg)
    src = JoystickSource(args.device or cfg.rc.device)
    print(f"{src}\nCtrl-C to quit.\n")
    if mapping.max_axis >= src.num_axes:
        sys.exit(f"drone.yaml maps axis {mapping.max_axis} but the device has {src.num_axes}")

    channels = sorted(cfg.rc.mapping)
    lines = len(channels) + 2
    first = True
    try:
        while src.alive:
            frame = src.latest()
            if frame is None:
                time.sleep(0.05)
                continue
            ticks = mapping.apply(frame.axes)
            out = []
            for ch in channels:
                entry = cfg.rc.mapping[ch]
                us = tick_to_us(ticks[ch])
                inv = " inv" if entry.invert else ""
                out.append(
                    f"CH{ch + 1:<2} axis {entry.axis}{inv:4} {us:6.0f}us {bar(us)}  "
                    f"{label_for(cfg, ch)}\033[K"
                )
            modes = active_modes(cfg, ticks)
            out.append(f"active modes: {', '.join(modes) or '(none)'}\033[K")
            pressed = [str(i) for i, b in enumerate(frame.buttons) if b]
            out.append(f"buttons:      {' '.join(pressed) or '(none)'}\033[K")
            if not first:
                print(f"\033[{lines}A", end="")
            print("\n".join(out), flush=True)
            first = False
            time.sleep(0.05)
        print(f"\nradio disconnected: {src.error}")
    except KeyboardInterrupt:
        print()
    finally:
        src.close()


if __name__ == "__main__":
    main()
