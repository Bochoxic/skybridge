# Improvements / TODO

Deferred work, roughly in order of value.

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
