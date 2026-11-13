# Improvements / TODO

Deferred work, roughly in order of value.

## RSSI decodes wrong (-208 dBm instead of ~-48)

Seen on hardware on 2026-11-07: `crsf_test.py` printed `RSSI=-208dBm` on
a good link (`LQ=100%`). ELRS sends RSSI as a **signed** byte that is
already negative (0xD0 = -48), but `decode_link_stats` in
`crsf/decoders.py` reads it unsigned and negates it (`-p[0]` → -208).
The CRSF spec instead says to send the positive magnitude (48 = -48 dBm),
so decode both conventions: a byte > 127 is signed (`b - 256`),
otherwise `-b`. This applies to `uplink_rssi1`, `uplink_rssi2` and
`downlink_rssi`. Add a test with both encodings.

Display/telemetry only: RC control is unaffected. `crsf_test.py` has the
same bug, but it is the frozen hardware baseline, so fix only the library.

## Live perception: still to do

Live tracking works on hardware (recorded flight 2026-11-08 18:46). Open:

**Before trusting the numbers**
- Measure `cam_height` and `cam_pitch` in `configs/perception.yaml`. They are
  still 0, so `z` is relative to the camera, not the floor. 1 degree of pitch
  error is about 3.5 cm at 2 m. Required before alt hold.
- Replay doesn't read `perception.yaml`, so `drone_detect.py` needs
  `--z-max 1.7` and the pose flags by hand. Have `--record` save the exact
  pipeline settings in the recording's `.json`, and have `drone_detect.py`
  load them, with flags still overriding. Older recordings keep the defaults.
- Replay a real flight through `drone_detect.py` (e.g.
  `2026-11-08_18-46-13_flight.rrd`) and compare with the states saved live.

**Not tested yet**
- The viewer dying mid-flight: close the Rerun window during a dry run and check
  the status line keeps updating. An earlier test of this was invalid.
- Viewer memory over flights of several minutes with the lighter live view.
  The old view passed its 2 GB limit in about 4 minutes.
- Battery and flight mode are missing from the 18:46 recording. The logging
  handles both, so check whether the drone is sending them.

**Performance**
- Detection is slower in flight than on recordings: about 13-20 ms per frame
  against 11 ms, p99 up to 57 ms. Probably CPU contention with the viewer;
  not confirmed. It runs at `nice 5` so that RC wins.
- `--record` writes depth from the detection loop. Its cost is unmeasured. If
  `skipped` climbs in the perception log, move it to its own thread.

**Rough edges**
- With no camera, `perception_live.py` prints a traceback and then doesn't
  exit. Make it exit with "no camera found", and show that reason in the
  `rc_fly.py` status line instead of only `VIS LOST (not running)`.
- A viewer started by standalone `perception_live.py` inherits its `nice 5`,
  and later flights reuse that slower viewer.

## RC / autonomy mux

`Bridge.set_channels` (raw passthrough) and the physical setters already
take over from each other. A mux layer would choose the source from a
radio switch, e.g. CH7 high = autonomy, with the radio always able to take
back control and the arm switch kept on the radio in both modes.

Altitude hold now does this for throttle: see `skybridge/control/` and
`rc_fly.py --control`. It runs after `cap_throttle()` and the arm guard, not
before the cap, which would rescale its output. A full autonomy mode would be
another controller with its own switch range and axes.

Altitude hold has been flown and holds height (first flights 2026-11-13).

## Radio liveness and reconnect

**Open risk with `--control`.** A frozen radio already holds the last sticks;
with a controller engaged it also holds the switch, so altitude hold keeps
running and the pilot can't end it from the radio. Ctrl-C on the laptop still
disarms, so keep someone at the keyboard until this is fixed.

joydev only reports changes, so a frozen radio that is still plugged in
looks the same as a quiet one. Only an unplug is detected. Reading the
HID reports (`/dev/hidraw*`, sent continuously, but root-only by default
and needing a udev rule) would give real liveness. After an unplug the
passthrough stops for good. Reconnecting would need the arm interlock
reset, so that the switch has to be seen OFF again.


## Betaflight CLI dump parser

**Why it matters:** `configs/drone.yaml` has to mirror the flight
controller's real settings. If it drifts — someone retunes rates in the
Configurator and forgets the YAML — every angle command is silently
wrong. There is no error signal from the drone.

Add a utility that reads a `diff all` / `dump` text file and emits the
YAML, so the two cannot disagree:

```
python -m skybridge.bfdump --dump my_quad_diff.txt --out configs/drone.yaml
```

Needs to extract `angle_limit`, `rates_type`, per-axis `rc_rate` /
`srate` / `expo`, `deadband`, `yaw_deadband`, `min_check`, and the `aux`
entries. Reading these live over MSP would be better still.

Partial mitigation already in place: `config.validate()` warns when the
rate curve is non-linear, which catches the most damaging kind of drift.

## Rates types beyond ACTUAL

Only `rates_type: actual` is supported. Betaflight also has BETAFLIGHT,
QUICK, RACEFLIGHT and KISS, each with different math. `config.py` rejects
them explicitly rather than computing something wrong.

`units.apply_actual_rates()` is the model to follow — transcribe the
corresponding `applyXxxRates()` from `fc/rc.c`.

## Throttle curve

Throttle is currently linear from `min_check` upward. Betaflight applies
a curve: on master it is a two-segment quadratic Bézier using
`thr_mid` / `thr_expo` / `thr_hover`; on 4.4 and earlier it was a 3-point
expo lookup with no `thr_hover`.

This only matters for precise thrust control — for a closed-loop
altitude controller the outer loop absorbs the nonlinearity.

## MAVLink telemetry as a second source

`scripts/wifi_telem.py` reads MAVLink over WiFi/UDP, which carries a
richer message set than CRSF (GPS, full sensor data). It could feed the
same `TelemetryState` alongside CRSF.

Tension to resolve: MAVLink over the backpack showed seconds of latency,
so merging naively would make `state.attitude` *worse*. Keep CRSF as the
low-latency source and expose MAVLink-only fields separately.

## Separate reader thread

The bridge currently does write and read on one thread, which is right at
50Hz and 115200 baud. If the RC rate or the telemetry volume goes up
enough that reads start eating the tick budget, split them.

`bridge.py` only ever calls `transport.write()` and
`transport.read_available()`, so this change is contained to
`transport.py`. Note that sharing a pyserial object across threads needs
care, and a naive lock is worse than no split — a blocking read holds the
lock and starves the RC writer.

## Unregistered telemetry frames

`decode_ahrs_accel` and `decode_ahrs_gyro` exist in the original script
but were never added to `DECODERS`, because the frame type IDs were
unknown. Confirm the IDs against a CRSF capture and register them.

Frames `0x2E` (ELRS_STATUS) and `0x3A` (ELRS_LINK) currently decode to a
raw hex dump.

## Arming preconditions

`arm()` checks that throttle is at minimum. Betaflight has more arming
gates — angle, gyro calibration, RX link. Reading `FLIGHT_MODE`
telemetry would let the bridge report *why* arming was refused rather
than leaving you to guess from the Configurator.
