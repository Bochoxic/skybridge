# skybridge

Control a drone from Python over ExpressLRS: RC commands out over CRSF,
telemetry back on the same link.

Commands are expressed in physical units — `set_angle(roll=10)` means ten
degrees, and the library computes the channel value that makes Betaflight
actually fly ten degrees.

## Quick start

```python
from skybridge import Bridge, DroneConfig

cfg = DroneConfig.from_yaml("configs/drone.yaml")

with Bridge(cfg) as bridge:          # RC starts flowing at 50Hz
    bridge.set_mode("angle")
    bridge.set_throttle(0.0)
    bridge.arm()

    while True:
        att = bridge.state.attitude
        if att and not att.is_stale(0.5):
            print(att.value.roll_deg)

        bridge.set_angle(roll=10)
        time.sleep(0.02)             # your rate, not the RC rate
```

Leaving the `with` block disarms and closes the port.

## Layout

```
src/skybridge/
  crsf/           protocol: framing, CRC, telemetry decoders
  units.py        physical units <-> channel values
  config.py       YAML -> validated dataclasses
  state.py        telemetry snapshot with per-field staleness
  transport.py    serial, plus a loopback for tests
  bridge.py       the Bridge class
  optitrack/  perception/  control/     (placeholders)

configs/drone.yaml    must mirror your Betaflight settings
scripts/              runnable scripts, incl. the original references
tests/                57 tests, no hardware needed
```

## Setup

```bash
uv sync --extra dev
uv run pytest
```

Hardware setup — wiring, baud rates, Betaflight and ExpressLRS
configuration — is documented in [MAVLINK.md](MAVLINK.md).

## Two things worth knowing

Both were found by reading Betaflight source, and both are easy to get
wrong.

### Angle mode is not linear in stick deflection

`pidLevel()` routes stick input through the *acro rate curve*:

```
angle = angle_limit * applyRates(deflection) / applyRates(1.0)
```

With stock rates (`rc_rate=7, srate=67`), asking for 20 degrees the naive
way commands **8 degrees**.

The curve collapses to linear when `rc_rate == srate`, because the
`stickMovement` term goes to zero and expo drops out entirely. **Set
`rc_rate = srate = 67`** in Betaflight — use 67 rather than 7, since both
are linear but 7 would cap acro at 70 deg/s instead of 670.

`config.validate()` warns if your YAML implies a non-linear curve. The
library inverts the curve numerically either way, but that inversion is
only correct while the YAML matches the FC.

### CRSF ticks are not what you would guess

Betaflight decodes ticks as `us = 0.62477 * tick + 881`, so tick 172 is
**988us, not 1000us**. Nominal endpoints land on ticks 190 / 991 / 1791.

Confirmed on hardware: sending 1000/1500/2000us reads back as
1000/1500/2000us in the Betaflight Receiver tab. Re-check on a new
airframe with:

```bash
python scripts/verify_channel_scale.py          # correct mapping
python scripts/verify_channel_scale.py --old    # the old assumption
```

Watch the Betaflight Configurator's Receiver tab — the correct mapping
reads 1000/1500/2000, the old one reads 988/1500/2012.

## Safety

- **Stale-command watchdog** — if control code stops calling `set_*` for
  `safety.command_timeout`, the bridge disarms immediately. A crashed
  control loop cannot leave the drone flying on its last input.
- **Arm guard** — `arm()` refuses unless throttle is at minimum.
- **Mode guard** — `set_angle()` while in acro raises `ModeMismatch`
  rather than reinterpreting degrees as deg/s.
- **Clamping** — angles to `safety.max_angle`, ticks to the valid range.
- **Clean shutdown** — closing the bridge disarms.

## Scripts

| Script | What it does |
|---|---|
| `example_control_loop.py` | Closed-loop example using the library |
| `verify_channel_scale.py` | Check tick↔us against the Receiver tab |
| `crsf_test.py` | Original CRSF reference (unchanged, still works) |
| `wifi_telem.py` | MAVLink telemetry over WiFi |
| `legacy/` | Abandoned RC-over-MAVLink attempts; see MAVLINK.md |

Deferred work is tracked in [IMPROVEMENTS.md](IMPROVEMENTS.md).
