#!/usr/bin/env python3
"""Fly from the radio, through the laptop: USB joystick in, CRSF out.

    Boxer --USB-C--> laptop --USB-UART--> ELRS TX module --2.4GHz--> drone

Sticks and switches are forwarded raw, so the drone flies exactly as it
does on the radio's own link (Betaflight applies rates and modes). The
laptop adds the guards in skybridge/passthrough.py:

  * arming is ignored until the arm switch has been seen OFF, so
    starting with the switch up never arms;
  * turning arm on with throttle up is refused until the switch is cycled;
  * --max-throttle scales the throttle stick (full stick = that fraction);
  * unplugging the radio stops the feed, and the bridge watchdog disarms
    within safety.command_timeout (0.2s).

Before the first run:
  1. The radio's INTERNAL RF must be OFF for this model, or two transmitters
     fight for the receiver. The USB joystick output keeps working.
  2. The receiver must be bound to the external TX module on serial.port.
  3. Run scripts/rc_probe.py and check that arm and mode switches show up.
  4. PROPS OFF until the Betaflight Receiver tab matches the sticks.

Ctrl-C stops and disarms.

Usage:
    python scripts/rc_fly.py --dry-run              # no TX: show what would be sent
    python scripts/rc_fly.py                        # live, full throttle range
    python scripts/rc_fly.py --max-throttle 0.5
"""

from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from skybridge import (  # noqa: E402
    Bridge,
    Bus,
    DroneConfig,
    LoopbackTransport,
    PassthroughStatus,
    RcPassthrough,
)
from skybridge.passthrough import throttle_fraction  # noqa: E402
from skybridge.rc import JoystickSource, active_modes  # noqa: E402
from skybridge.units import tick_to_us  # noqa: E402

CONFIG = Path(__file__).parent.parent / "configs" / "drone.yaml"


def status_line(bridge: Bridge, cfg: DroneConfig, st: PassthroughStatus) -> str:
    if st.state != "live":
        return f"\r{st.state.upper():6} {st.note}\033[K"

    cm = cfg.channels
    t = st.ticks
    flag = "FAILSF" if bridge.failsafe_active else ("ARMED " if st.armed else "disarm")
    sticks = (
        f"T{throttle_fraction(t[cm.throttle]) * 100:3.0f}% "
        f"R{tick_to_us(t[cm.roll]):5.0f} P{tick_to_us(t[cm.pitch]):5.0f} "
        f"Y{tick_to_us(t[cm.yaw]):5.0f}"
    )
    modes = ",".join(m for m in active_modes(cfg, t) if m != "arm") or "-"

    s = bridge.state
    tel = []
    if s.attitude and not s.attitude.is_stale(0.5):
        a = s.attitude.value
        tel.append(f"att {a.roll_deg:+5.1f}/{a.pitch_deg:+5.1f}")
    elif s.attitude:
        tel.append(f"att STALE {s.attitude.age:.1f}s")
    else:
        tel.append("no telemetry")
    if s.battery:
        tel.append(f"{s.battery.value.voltage_V:4.1f}V")
    if s.link:
        tel.append(f"LQ{s.link.value.uplink_lq}")

    note = f"  !! {st.note}" if st.note else ""
    return f"\r{flag} {sticks} mode={modes} | {' '.join(tel)}{note}\033[K"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--device", help="override rc.device (joystick)")
    ap.add_argument("--port", help="override serial.port (TX module)")
    ap.add_argument("--max-throttle", type=float, help="override rc.max_throttle (0..1]")
    ap.add_argument("--dry-run", action="store_true",
                    help="no TX module: frames go to an in-memory transport")
    args = ap.parse_args()

    cfg = DroneConfig.from_yaml(args.config)
    bus = Bus()  # the recording hook: subscribe here to log rc/telemetry later

    source = JoystickSource(args.device or cfg.rc.device, bus=bus)
    print(source)

    transport = LoopbackTransport() if args.dry_run else None
    if args.port and not args.dry_run:
        from skybridge import SerialTransport
        transport = SerialTransport(args.port, cfg.serial.baud)
    bridge = Bridge(cfg, transport, bus=bus)
    port = args.port or cfg.serial.port
    print("DRY RUN: nothing is transmitted" if args.dry_run else f"TX module on {port}")

    passthrough = RcPassthrough(bridge, source, cfg, max_throttle=args.max_throttle)
    if passthrough.mapping.max_axis >= source.num_axes:
        sys.exit(f"drone.yaml maps axis {passthrough.mapping.max_axis}, "
                 f"device has {source.num_axes}")
    print(f"max throttle {passthrough.max_throttle:.0%}. Ctrl-C to stop and disarm.\n")

    stop = threading.Event()
    last = [""]

    def show(st: PassthroughStatus) -> None:
        if transport is not None and isinstance(transport, LoopbackTransport):
            transport.written.clear()  # dry run: don't accumulate forever
        line = status_line(bridge, cfg, st)
        if line != last[0]:
            print(line, end="", flush=True)
            last[0] = line
        if st.state == "dead":
            stop.set()

    try:
        with bridge:
            passthrough.run(stop, show)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        print("\nstopped, disarmed.")
        if not source.alive and source.error:
            print(f"radio: {source.error}")


if __name__ == "__main__":
    main()
