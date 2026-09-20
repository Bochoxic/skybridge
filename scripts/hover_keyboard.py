#!/usr/bin/env python3
"""Manual throttle via the keyboard: ramp up to a hover, hold, cut.

REMOVE THE PROPS for the first run. Only fly this once you have watched
the throttle number climb and the motors respond in the way you expect.

Why a ramp and not a step: the up/down arrows change the throttle *rate*,
so holding Up climbs at --ramp per second rather than jumping. Releasing
holds the current value, which is what you want for trimming a hover.

The loop must call a set_* method every iteration whether or not a key
was pressed -- `safety.command_timeout` disarms after 0.2s of silence --
so keys are read non-blocking from a raw-mode terminal.

Throttle latches and roll does not, which is deliberate: you want a
hover to hold itself while your hands are busy, but a bank angle that
outlived the keypress would fly the drone into a wall.

Keys:
    up / down     throttle ramp up / down (hold to keep ramping)
    left / right  roll --roll degrees while held, 0 when released
    space         CUT: throttle 0 and disarm immediately
    q / Ctrl-C    cut and quit

Hover on this airframe is ~0.30 throttle (measured 2026-09-20), so the
0.4 default ceiling leaves a third above hover to correct a sag while
capping a runaway well short of full power. Re-measure after any change
to weight, props or battery -- hover throttle moves first.

Usage:
    python scripts/hover_keyboard.py                  # sticks only, no arm
    python scripts/hover_keyboard.py --arm            # arms, props off!
    python scripts/hover_keyboard.py --arm --max-throttle 0.5
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from skybridge import Bridge, DroneConfig  # noqa: E402

CONFIG = Path(__file__).parent.parent / "configs" / "drone.yaml"

LOOP_HZ = 50.0

# How long a single arrow keypress keeps the ramp alive. Terminals send
# no key-release event, and auto-repeat is ~30Hz once it kicks in, so a
# held key is indistinguishable from a rapid series of presses. This
# window is the bridge between the two: long enough to survive the gap
# between repeats, short enough that letting go stops the climb promptly.
#
# The cost is that throttle coasts for up to KEY_HOLD after you let go --
# 0.015 units at the default ramp, and proportionally more if you raise
# --ramp. Trim the last few percent with taps rather than a long hold,
# and use space for anything urgent; it cuts on the same tick.
KEY_HOLD = 0.12

ARROWS = {"\x1b[A": "up", "\x1b[B": "down", "\x1b[C": "right", "\x1b[D": "left"}


class RawTerminal:
    """Put stdin in raw mode so keys arrive without waiting for Enter.

    Restored in __exit__ whatever happens -- leaving a terminal in raw
    mode after a crash is a genuinely unpleasant thing to do to someone
    who is standing next to an armed drone.
    """

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._saved: list | None = None

    def __enter__(self) -> RawTerminal:
        self._saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc: object) -> None:
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)

    def read_keys(self) -> list[str]:
        """Every key waiting right now, as names. Never blocks.

        Arrows arrive as the three bytes ESC [ A..D; they are decoded
        here rather than surfaced raw so the caller deals in names only.
        """
        if not select.select([sys.stdin], [], [], 0)[0]:
            return []

        data = sys.stdin.read(1)
        while select.select([sys.stdin], [], [], 0)[0]:
            data += sys.stdin.read(1)

        keys: list[str] = []
        i = 0
        while i < len(data):
            if data[i] == "\x1b" and data[i : i + 3] in ARROWS:
                keys.append(ARROWS[data[i : i + 3]])
                i += 3
            else:
                keys.append(data[i])
                i += 1
        return keys


def status_line(
    bridge: Bridge, throttle: float, armed: bool, roll: float = 0.0
) -> str:
    att = bridge.state.attitude
    if att is None:
        attitude = "no telemetry"
    elif att.is_stale(0.5):
        attitude = f"STALE {att.age:.1f}s"
    else:
        attitude = (
            f"roll={att.value.roll_deg:+5.1f} pitch={att.value.pitch_deg:+5.1f}"
        )

    alt = bridge.state.altitude
    alt_s = f" alt={alt.value.alt_m:+5.2f}m" if alt else ""

    batt = bridge.state.battery
    batt_s = f" {batt.value.voltage_V:4.1f}V" if batt else ""

    bar_width = 24
    filled = int(throttle * bar_width)
    bar = "#" * filled + "-" * (bar_width - filled)

    flag = "ARMED " if armed else "disarm"
    if bridge.failsafe_active:
        flag = "FAILSF"

    # Commanded roll next to the measured roll inside `attitude`, so a
    # drone that is not following the command is obvious at a glance.
    if roll > 0:
        cmd = f" roll>{roll:+4.1f}"
    elif roll < 0:
        cmd = f" roll<{roll:+4.1f}"
    else:
        cmd = " roll  0.0"

    return (
        f"\r{flag} [{bar}] {throttle * 100:5.1f}%{cmd} | "
        f"{attitude}{alt_s}{batt_s}\033[K"
    )


def countdown(bridge: Bridge, seconds: int) -> None:
    """Hold sticks neutral for `seconds`, keeping the watchdog fed."""
    deadline = time.monotonic() + seconds
    while (left := deadline - time.monotonic()) > 0:
        bridge.set_angle(roll=0.0, pitch=0.0, yaw_rate=0.0)
        bridge.set_throttle(0.0)
        print(
            f"\rarming in {left:3.1f}s  (Ctrl-C aborts) | "
            f"{status_line(bridge, 0.0, False).lstrip(chr(13))}",
            end="",
            flush=True,
        )
        time.sleep(1.0 / LOOP_HZ)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", action="store_true", help="actually arm (props off!)")
    ap.add_argument(
        "--max-throttle",
        type=float,
        default=0.4,
        help="hard ceiling on commanded throttle (default 0.4)",
    )
    ap.add_argument(
        "--ramp",
        type=float,
        default=0.15,
        help="throttle units per second while an arrow is held (default 0.15)",
    )
    ap.add_argument(
        "--roll",
        type=float,
        default=2.0,
        help="degrees of roll while left/right is held (default 2.0)",
    )
    ap.add_argument("--countdown", type=int, default=5)
    ap.add_argument("--config", default=str(CONFIG))
    args = ap.parse_args()

    cfg = DroneConfig.from_yaml(args.config)

    # set_angle() would clamp this silently; saying so beats having the
    # drone bank less than the number on screen claims.
    if abs(args.roll) > cfg.safety.max_angle:
        ap.error(
            f"--roll {args.roll} exceeds safety.max_angle "
            f"({cfg.safety.max_angle}) in {args.config}"
        )

    print("=" * 64)
    print("  KEYBOARD HOVER TEST -- REMOVE THE PROPS FOR THE FIRST RUN")
    print("=" * 64)
    print(f"  port          {cfg.serial.port} @ {cfg.serial.baud}")
    print(f"  max throttle  {args.max_throttle:.2f}")
    print(f"  ramp          {args.ramp:.2f} / second")
    print(f"  roll          {args.roll:.1f} deg (max_angle {cfg.safety.max_angle})")
    print(f"  watchdog      disarms after {cfg.safety.command_timeout:.1f}s idle")
    print()
    print("  up/down     ramp throttle (latches)    space   CUT + disarm")
    print(f"  left/right  roll {args.roll:.1f} deg (momentary)   q       cut and quit")
    print("=" * 64)
    print()

    throttle = 0.0
    armed = False

    with Bridge(cfg) as bridge, RawTerminal() as term:
        bridge.set_mode("angle")
        bridge.set_throttle(0.0)

        if args.arm:
            try:
                countdown(bridge, args.countdown)
            except KeyboardInterrupt:
                print("\nAborted before arming.")
                return

            print("\n")
            if bridge.state.attitude is None:
                print("No telemetry -- is the drone powered and linked?")
                print("Aborting rather than arming blind.")
                return

            bridge.arm()
            armed = True
            print(">>> ARMED <<<\n")
        else:
            print("Not arming (pass --arm). Throttle is commanded but the FC")
            print("will ignore it while disarmed.\n")

        period = 1.0 / LOOP_HZ
        ramp_dir = 0.0
        ramp_until = 0.0
        roll_dir = 0.0
        roll_until = 0.0
        last = time.monotonic()

        try:
            while True:
                now = time.monotonic()
                dt = now - last
                last = now

                for key in term.read_keys():
                    if key == "up":
                        ramp_dir, ramp_until = 1.0, now + KEY_HOLD
                    elif key == "down":
                        ramp_dir, ramp_until = -1.0, now + KEY_HOLD
                    elif key == "right":
                        roll_dir, roll_until = 1.0, now + KEY_HOLD
                    elif key == "left":
                        roll_dir, roll_until = -1.0, now + KEY_HOLD
                    elif key == " ":
                        throttle = 0.0
                        bridge.disarm()
                        armed = False
                        ramp_dir = 0.0
                        roll_dir = 0.0
                    elif key in ("q", "\x03"):
                        raise KeyboardInterrupt

                # Credit only the slice of dt that fell inside the hold
                # window. Applying the whole step whenever the window was
                # live at its start overshoots by up to one tick's worth
                # of climb every time you let go of the key.
                active = max(0.0, min(dt, ramp_until - (now - dt)))
                throttle += ramp_dir * args.ramp * active
                throttle = max(0.0, min(args.max_throttle, throttle))
                if now > ramp_until:
                    ramp_dir = 0.0

                # Roll is momentary, unlike throttle: the angle applies
                # only while the key is held and snaps back to level the
                # moment the window lapses. A roll that latched the way
                # throttle does would leave the drone banked and
                # accelerating sideways after you stopped pressing.
                if now > roll_until:
                    roll_dir = 0.0
                roll = roll_dir * args.roll

                # Every iteration, unconditionally: the watchdog counts
                # from the last set_* call, not from the last keypress.
                bridge.set_angle(roll=roll, pitch=0.0, yaw_rate=0.0)
                bridge.set_throttle(throttle)

                print(
                    status_line(bridge, throttle, armed, roll), end="", flush=True
                )
                time.sleep(period)

        except KeyboardInterrupt:
            pass
        finally:
            bridge.set_throttle(0.0)
            bridge.disarm()
            print("\n\n>>> DISARMED <<<")
            time.sleep(0.3)

    print("Bridge closed.")


if __name__ == "__main__":
    main()
