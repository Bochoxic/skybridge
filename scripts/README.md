# Scripts

Runnable tools for bringing up, testing and flying the drone. Every
script here talks to real hardware.

Every script that arms disarms on exit — including on Ctrl-C and on an
unhandled exception.

`arm_test.py`, `hover_keyboard.py` and `disarm.py` take `--config`
(default `configs/drone.yaml`). `example_control_loop.py` reads that path
but has no flag for it; `crsf_test.py` and `verify_channel_scale.py`
predate the config and take the port on the command line instead.

```bash
uv run python scripts/<name>.py --help
```

> **Props off.** Every script that can arm says so in its own header. Work
> through them in the order below the first time: each one assumes the
> previous one behaved.

---

## Bring-up order

The scripts form a ladder. Each rung proves one thing, so that when
something fails you know which layer broke.

| # | Script | Proves | Arms? |
|---|---|---|---|
| 1 | `crsf_test.py` | The serial link works and telemetry decodes | no |
| 2 | `verify_channel_scale.py` | Your microsecond values arrive intact | no |
| 3 | `arm_test.py` | The FC accepts arm/disarm over the link | **yes** |
| 4 | `hover_keyboard.py` | Manual throttle and roll under your hand | **yes** |
| 5 | `example_control_loop.py` | Closed-loop code can fly it | **yes** |

`disarm.py` is the panic button and belongs open in a second terminal
from step 3 onwards.

---

## `crsf_test.py` — link and telemetry check

The original reference implementation, self-contained (no `skybridge`
import) and deliberately left unchanged because it is proven on hardware.
When the library misbehaves, this is the known-good baseline to compare
against.

Sends RC at 50Hz with **throttle low and ARM off**, sweeping yaw as a
slow sine so you can see stick movement in the Betaflight Receiver tab.
Decodes and prints every telemetry frame that comes back.

```bash
uv run python scripts/crsf_test.py                    # /dev/ttyUSB0 @ 115200
uv run python scripts/crsf_test.py /dev/ttyACM0 420000
```

Takes positional `port` and `baud` — not `--config`.

**What good looks like:** a live line of roll/pitch/yaw that responds when
you tilt the airframe, `LQ=100%`, and a rate around 40–50Hz.

**If the line never appears,** telemetry is not arriving: check the port,
the baud (see [MAVLINK.md](../MAVLINK.md) — the USB-UART path is 115200,
*not* 420000), and that the drone is powered and bound.

---

## `verify_channel_scale.py` — tick↔microsecond check

Steps one channel through 1000/1200/1500/1800/2000us, holding each for
four seconds so you can read it in the **Betaflight Configurator →
Receiver tab** and confirm the number matches.

This exists because the mapping is genuinely counterintuitive: Betaflight
decodes ticks as `us = 0.62477 * tick + 881`, so the widely-copied
"1000us = tick 172" is wrong by 12us at the endpoints. Run it with
`--old` to watch that error appear.

```bash
uv run python scripts/verify_channel_scale.py          # correct mapping
uv run python scripts/verify_channel_scale.py --old    # the old assumption
uv run python scripts/verify_channel_scale.py --channel 1   # drive pitch
```

**What good looks like:** the Receiver tab reads 1000/1500/2000 exactly.
With `--old` it reads 988/1500/2012.

Worth re-running on a new airframe or after a Betaflight update.

---

## `arm_test.py` — first-light arming check

**Props off.** Arms the drone for a few seconds at zero throttle, prints
telemetry throughout, then disarms. Motors should not spin.

Refuses to arm if no telemetry has arrived, rather than arming blind.

```bash
uv run python scripts/arm_test.py                 # 3 seconds armed
uv run python scripts/arm_test.py --seconds 5
uv run python scripts/arm_test.py --countdown 10  # longer to back out
```

**What good looks like:** `>>> ARMED <<<`, the flight mode in telemetry
changing, and a clean `>>> DISARMED <<<`.

**If arming is refused,** the FC is rejecting it, not the script — check
the arm switch range in `configs/drone.yaml` against your Betaflight `aux`
entries, and that throttle reads below `min_check` (1050) in the Receiver
tab.

---

## `hover_keyboard.py` — manual flight from the keyboard

**Props off for the first run.** Fly the drone by hand to find its hover
throttle before trusting a controller with it.

| Key | Action |
|---|---|
| ↑ / ↓ | Ramp throttle up/down — **latches** at the value you release on |
| ← / → | Roll ∓ `--roll` degrees while held — **returns to 0** on release |
| space | CUT: throttle to zero and disarm, immediately |
| `q` / Ctrl-C | Cut and quit |

The asymmetry is deliberate. A hover should hold itself while your hands
are off the keys, but a bank angle that outlived the keypress would keep
accelerating the drone sideways.

Throttle moves as a *rate*, not a step: holding ↑ climbs at `--ramp` per
second, so a mistyped key cannot jump to full power. `--max-throttle` is
a hard ceiling on top of that.

```bash
uv run python scripts/hover_keyboard.py                        # no arming
uv run python scripts/hover_keyboard.py --arm                  # props off!
uv run python scripts/hover_keyboard.py --arm --max-throttle 0.5
uv run python scripts/hover_keyboard.py --arm --roll 5 --ramp 0.1
```

`--roll` is validated against `safety.max_angle` at startup, so the
on-screen number can never overstate the bank the FC will fly.

The status line shows commanded roll beside measured roll:

```
ARMED  [#######-----------------]  30.0% roll>+2.0 | roll= +1.8 pitch= -0.2 alt=+0.41m 15.8V
```

Watching those two converge *is* the test. If `roll>+2.0` shows while
measured roll stays flat, the command is not reaching the FC.

**One caveat:** throttle coasts for up to 0.12s after you let go (≈1.5% at
the default ramp, proportionally more if you raise `--ramp`). Terminals
send no key-release event, so a held key is indistinguishable from fast
auto-repeat, and that window is what bridges the gap between repeats.
Trim the last few percent with taps rather than a long hold. Roll has no
coast — it cuts to zero on the same tick.

**Hover throttle on this airframe is ≈0.30** (measured 2026-09-20). The
default `--max-throttle 0.4` sits a third above that — enough to correct
a sag, not enough for a surprise climb. Raise it only with a reason.

From zero, the default ramp reaches hover in about two seconds. Creep the
last few percent in taps rather than one long hold, so the 0.12s coast
below cannot carry you past it.

Re-measure after any change to weight, props or battery chemistry: hover
throttle is the first thing that moves.

---

## `example_control_loop.py` — closed-loop template

The intended shape of control code: the bridge keeps RC flowing at 50Hz
on its own thread while your loop runs at whatever rate it likes and
calls `set_angle()` / `set_throttle()` when it has something to say.

Flies a gentle ±2° roll oscillation at 0.5Hz. Copy this file as the
starting point for a real controller.

```bash
uv run python scripts/example_control_loop.py           # sticks only
uv run python scripts/example_control_loop.py --arm     # props off!
uv run python scripts/example_control_loop.py --arm --seconds 30
```

Note the countdown loop keeps calling `set_angle()` while counting down —
a bare `sleep()` there would let the stale-command watchdog trip, and
`arm()` would fire into a failsafe that cancels it immediately. Any
control code you write has the same obligation: **call a `set_*` method
every iteration**, whether or not you have a new command.

---

## `disarm.py` — panic button

Run this when something has gone wrong: a control script crashed, or the
drone is armed and you want it stopped now. Holds ARM low and throttle
minimum for a couple of seconds so the FC definitely sees it.

```bash
uv run python scripts/disarm.py
uv run python scripts/disarm.py --hold 10
```

Keep a terminal with this command ready to run whenever you are armed.

It needs the serial port, so it cannot run while another script holds it
— kill that script first (Ctrl-C, which already disarms on the way out).

---

## `wifi_telem.py` — MAVLink telemetry over WiFi

Separate path from everything above: reads MAVLink from the ExpressLRS TX
Backpack over UDP instead of CRSF over serial. Useful for telemetry
without occupying the serial port, but it is **read-only** — there is no
RC control here.

```bash
uv run python scripts/wifi_telem.py           # listen on 14550
uv run python scripts/wifi_telem.py 14550 14555
```

Setup: ELRS Lua script → Backpack → Telemetry → WiFi, then join the
`ExpressLRS TX Backpack XXXXXX` network (password `expresslrs`).

The script sends its own heartbeats to the backpack's listen port
(14555), because the backpack broadcasts only until a ground station
talks to it and then latches onto that IP.

---

## `legacy/`

Abandoned attempts at RC-over-MAVLink (`rc_only.py`, `test.py`,
`test_rc.py`), kept as a record of what did not work and why.

`RC_CHANNELS_OVERRIDE` is not a viable control path on this stack — the
reasoning is in [MAVLINK.md](../MAVLINK.md). Do not build on these; use
the CRSF path.
