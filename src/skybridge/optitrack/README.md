# optitrack — RealSense ground truth

Tools for finding and tracking the drone from an external Intel RealSense
D455. The goal is a ground-truth position/velocity estimate to compare
against (and eventually feed) the flight controller. Everything here is
**offline**: record first, then run detection on the recording.

> The package is named `optitrack`, but no OptiTrack hardware is involved
> yet — the RealSense is the sensor doing the tracking.

## Pipeline

```
realsense_record.py ──► data/realsense/<stem>.rrd + <stem>.json
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
      drone_detect.py (3D, depth)     motion_detect.py (2D, RGB)
      <stem>_detections.csv/.rrd      <stem>_motion.csv/.rrd
      <stem>_predictions.csv
```

Both detectors log under the same Rerun recording id, so their outputs open
together on one timeline:

```bash
rerun <stem>_detections.rrd <stem>_motion.rrd
```

## Files

| File | What it does |
|---|---|
| `realsense_record.py` | Record RGB + depth + IMU from the D455 to a Rerun `.rrd`, plus a JSON sidecar with intrinsics/extrinsics and depth scale |
| `drone_detect.py` | Depth-based 3D detection (background subtraction → crop box → DBSCAN) with a constant-velocity Kalman tracker |
| `motion_detect.py` | 2D motion detection on the RGB stream (MOG2 background subtraction → largest blob). Not fused with depth yet |
| `../../../tests/test_drone_tracker.py` | Tracker behaviour across depth dropouts (coasting, sparse fallback, reset) |

Each script's module docstring is the full reference: every output column,
every Rerun entity, and the reasoning behind each step. `--help` lists all
options. This README covers the workflow and the knobs you'll actually touch.

## 1. Record

```bash
uv run src/skybridge/optitrack/realsense_record.py --label hover_test --seconds 30
```

- Writes `data/realsense/YYYY-MM-DD_HH-MM-SS_<label>.rrd` and `.json`
  (git-ignored — recordings are hundreds of MB).
- `--no-viewer` skips the live Rerun window; `--no-imu` records color+depth only.
- Defaults: 848×480 @ 30 fps, 20 s.
- Two timelines: `sensor_time` (s since first frame) and `wall_time`
  (Unix time, for aligning with flight-controller telemetry).

**Start each take with the drone out of view.** `drone_detect.py` learns
the static background from the first frames (`--bg-frames`, default 30 ≈ 1 s).

## 2. Detect in 3D (depth)

```bash
uv run src/skybridge/optitrack/drone_detect.py data/realsense/<stem>.rrd

# quick check on the first 100 frames, watching live
uv run src/skybridge/optitrack/drone_detect.py data/realsense/<stem>.rrd --max-frames 100 --viewer
```

The `.json` sidecar must sit next to the `.rrd`.

Per depth frame:

1. **Foreground** — pixels at least `--bg-delta` (+ `--bg-delta-rel`·depth²)
   closer than the learned background. Walls and floor drop out here.
2. **Back-project** to 3D in the camera frame and keep points inside the
   crop box (`--x-min` … `--z-max`, the flight volume).
3. **Cluster** with DBSCAN (`--eps`, `--min-samples`); keep drone-sized
   clusters (`--min-size` … `--max-size`).
4. **Track** with a constant-velocity Kalman filter (x, y, z, vx, vy, vz):
   - With a confirmed track, take the cluster nearest the *prediction*
     within `--gate`. If no cluster forms, fall back to sparse box points
     near the prediction (`--sparse-*`, trusted less). With nothing at all,
     coast on the prediction until `--max-coast`, then drop the track.
   - With no track, a candidate must persist for `--persist` frames, and is
     rejected as ambiguous if a second cluster is comparable
     (`--ambiguity-ratio`).

Each CSV row has a `status`: `warmup | none | ambiguous | candidate |
detected | coasting | lost`, plus a `reason` explaining the association.

### Frames

| Frame | Axes | Used for |
|---|---|---|
| camera | x right, y down, z forward (RealSense optical) | detection, crop box |
| world | x right, y forward, z up; origin on the floor below the camera | CSV positions/velocities, viewer |

Set the camera pose with `--cam-height` (m, lens above floor) and
`--cam-pitch` (deg, positive = looking down). With the defaults (0, 0),
world z is height relative to the camera.

### Knobs worth knowing

| Option | Default | Turn it when |
|---|---|---|
| `--x/y/z-min/max` | ±1, ±1, 0.3–2.0 m | Clutter inside the box, or the drone leaves it |
| `--bg-delta` | 0.10 m | Drone near a wall is missed (lower) / noise shows up as foreground (raise) |
| `--gate` | 0.5 m | Track jumps to clutter (lower) / fast flight loses the track (raise) |
| `--max-coast` | 1.5 s | How long to survive depth dropouts before resetting |
| `--meas-std`, `--accel-std` | 0.02 m, 3 m/s² | Filter too laggy (raise accel) / too jittery (raise meas) |
| `--blend-ms` | 100 ms | Smoothing of the output position only; the filter is unaffected |
| `--predict-hz` | 90 Hz | Rate of between-frame predictions in `_predictions.csv` |
| `--no-bg` | off | Compare against the old no-background, largest-cluster behaviour |

### Viewer

Red = raw detections, blue = Kalman estimate (magenta where it's
prediction-only), white = smoothed output, magenta sphere = gate around the
prediction. Clusters are coloured by verdict: green ok, yellow
`few_points`, cyan `too_small`, purple `too_big`, red `outside_gate`. The
"association log" panel says why each unassociated frame failed. Trail
length is a blueprint setting — change it live under *Visible time range*.

## 3. Detect in 2D (RGB motion)

```bash
uv run src/skybridge/optitrack/motion_detect.py data/realsense/<stem>.rrd
```

The camera is static, so changing pixels are the drone (or its shadow or
reflections). MOG2 background subtraction → open/close morphology →
connected components above `--min-area` px → largest blob is the
candidate, with centroid (u, v) and bbox.

- `--var-threshold` (25): raise if the background flickers into the mask.
- `--learning-rate` (-1 = auto): lower it so a hovering drone doesn't fade
  into the background.
- `--warmup` (30 frames) before blobs are reported.

This is image-only and not yet fused with the depth detections.

## Tests

```bash
uv run pytest tests/test_drone_tracker.py
```

On machines with ROS Humble sourced, ROS's pytest plugins break collection
(`No module named 'lark'`). Run with `env -u PYTHONPATH uv run pytest`.
