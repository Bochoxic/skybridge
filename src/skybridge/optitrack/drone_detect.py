# -*- coding: utf-8 -*-
"""
Offline drone detection from a RealSense depth recording made with
realsense_record.py, plus a constant-velocity Kalman filter that turns the
detections into smoothed position and velocity.

Pipeline:
    0. warm-up: the first --bg-frames depth frames build a per-pixel static
       background (median depth). The drone must be out of view meanwhile.
Then per depth frame:
    1. foreground = pixels at least --bg-delta closer than the background
       (or valid where the background had no depth). Walls, floor and other
       static surfaces are removed here, so they can't become the drone.
    2. back-project foreground pixels to 3D points in the camera frame
       (RealSense depth optical frame: x right, y down, z forward, metres)
    3. keep only points inside a box (the flight volume)
    4. DBSCAN-cluster what is left; keep clusters of drone-like size
       (--min-size .. --max-size)
    5. a constant-velocity Kalman filter (state x,y,z,vx,vy,vz) predicts
       every frame. Association:
       - confirmed track (filter alive): take the cluster nearest the
         filter's *prediction*, within --gate, and accept it straight away.
         If no cluster forms but >= --sparse-min-points box points lie within
         --sparse-radius of the prediction (too sparse for DBSCAN), they are
         used as a less trusted measurement (reason ok_sparse).
         Frames with nothing to associate coast on the prediction; the track
         survives depth dropouts up to --max-coast seconds, then resets.
       - no track: take the cluster nearest the previous candidate (within
         --gate), or the largest one, unless a second cluster is comparable
         (--ambiguity-ratio) -> ambiguous. A candidate seen in --persist
         consecutive frames confirms a new track and starts the filter.

Each CSV row has a status:
    warmup | none | ambiguous | candidate | detected | coasting | lost
(lost = coasted longer than --max-coast, the track is dropped)
--no-bg restores the old behaviour (no background, largest cluster wins)
for comparison.

Run:
    uv run drone_detect.py ../../../data/realsense/2026-10-28_20-40-52_moving_drone.rrd

    # quick test on the first 100 frames, watching live:
    uv run drone_detect.py <recording>.rrd --max-frames 100 --viewer

Outputs, next to the recording:
    <stem>_detections.csv  -- one row per depth frame
    <stem>_detections.rrd  -- depth, cropped points, crop box, drone centroid
                              (view with: rerun <stem>_detections.rrd)

Frames:
    camera  RealSense optical frame: x right, y down, z forward (depth).
            Detection runs here; the crop box (--x-min .. --z-max) is in it.
    world   x right, y forward (horizontal), z up (height); origin on the
            floor straight below the camera. Set the camera pose with
            --cam-height (lens above the floor) and --cam-pitch (degrees
            tilted down). With the defaults (0, 0) world z is height relative
            to the camera and y is horizontal distance in front of it.
CSV x,y,z (raw centroid), fx,fy,fz (filtered), ox,oy,oz (smoothed output)
and vx,vy,vz are in world; reason, since_update, gate, near_pts, near_dist,
cand_n, cand_extent, cand_dist explain the association (see REASONS);
cam_x,cam_y,cam_z and size_* are the raw centroid / extent in camera.

In the viewer (Z up): world/camera/* holds the camera-frame data (crop box,
subtracted background, cropped points, cluster), placed in the world by the
camera pose. In world coordinates:
    world/drone           raw detection this frame (red)
    world/trail/raw       raw detections, last --trail-seconds (red dots)
    world/filtered        Kalman position + velocity arrow (blue)
    world/trail/filtered  Kalman path, last --trail-seconds; blue where a
                          detection updated it, magenta = prediction only
    world/prediction      this frame's predicted position with the --gate
                          sphere the next cluster must fall into (magenta)
    world/output          smoothed output position (white): filter + blended
                          corrections (--blend-ms), at --predict-hz between frames
    world/trail/output    its path, last --trail-seconds
    world/camera/clusters every DBSCAN cluster, coloured and labelled by
                          verdict: green ok, yellow few_points, cyan too_small,
                          purple too_big, red outside_gate (d = distance to the
                          prediction / previous candidate)
    world/trajectory      whole flight, hidden by default (tick it to show)
The "association log" panel says why each non-associated frame failed (see
REASONS below), and plots/association shows cand_dist vs gate, the
closest box point to the prediction, points within the gate and time since
the last update vs --max-coast.
Trails are windowed by Rerun at playback time (blueprint "visible time
range"), so they are not re-logged every frame; adjust the window live by
selecting the entity -> Visible time range.
camera/foreground is the per-frame foreground mask; plots/ holds raw vs
filtered world position and velocity (z = height).
"""
import argparse
import collections
import csv
import json
import sys
import time
import warnings
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import rerun.chunk as rrc
from sklearn.cluster import DBSCAN

CSV_FIELDS = ["frame", "wall_time", "sensor_time", "status", "n_foreground",
              "n_cropped", "n_clusters", "detected",
              "x", "y", "z", "cam_x", "cam_y", "cam_z",
              "n_points", "size_x", "size_y", "size_z",
              "fx", "fy", "fz", "vx", "vy", "vz", "speed",
              "ox", "oy", "oz",
              "reason", "since_update", "gate", "near_pts", "near_dist",
              "cand_n", "cand_extent", "cand_dist"]
PRED_FIELDS = ["sensor_time", "kind", "x", "y", "z", "vx", "vy", "vz"]
VERDICT_COLORS = {"ok": [0, 220, 0], "few_points": [255, 220, 0],
                  "too_small": [0, 220, 220], "too_big": [150, 100, 255],
                  "outside_gate": [255, 60, 60]}


def iter_depth_frames(rrd_path):
    """Yield (wall_time_s, sensor_time_s, depth uint16 HxW) for every depth frame."""
    for chunk in rrc.RrdReader(str(rrd_path)).stream():
        if chunk.entity_path != "/camera/depth" or chunk.is_static:
            continue
        rb = chunk.to_record_batch()
        wall = rb.column("wall_time").cast("int64").to_numpy() / 1e9
        sensor = rb.column("sensor_time").cast("int64").to_numpy() / 1e9
        buffers = rb.column("DepthImage:buffer").to_pylist()
        formats = rb.column("DepthImage:format").to_pylist()
        for i in range(rb.num_rows):
            if not buffers[i]:
                continue
            fmt = formats[i][0]
            depth = np.frombuffer(bytes(buffers[i][0]), dtype=np.uint16)
            yield wall[i], sensor[i], depth.reshape(fmt["height"], fmt["width"])


class BackProjector:
    """Depth image -> Nx3 points (metres, camera frame), on a precomputed pixel grid."""

    def __init__(self, intr, depth_scale, stride):
        self.stride = stride
        self.scale = depth_scale
        u = np.arange(0, intr["width"], stride)
        v = np.arange(0, intr["height"], stride)
        uu, vv = np.meshgrid(u, v)
        self.kx = ((uu - intr["ppx"]) / intr["fx"]).ravel()
        self.ky = ((vv - intr["ppy"]) / intr["fy"]).ravel()

    def __call__(self, depth, mask=None):
        """mask: optional flat bool array over the strided grid, AND-ed with valid."""
        z = depth[::self.stride, ::self.stride].ravel() * self.scale
        valid = z > 0
        if mask is not None:
            valid &= mask
        return self.points(z, valid)

    def points(self, z, keep):
        """Flat strided depth in metres + flat bool keep mask -> Nx3 points."""
        z = z[keep]
        return np.column_stack((self.kx[keep] * z, self.ky[keep] * z, z))


class DepthBackground:
    """Per-pixel static depth background from warm-up frames, on the strided grid."""

    def __init__(self, depth_scale, stride, delta, delta_rel):
        self.scale = depth_scale
        self.stride = stride
        self.delta = delta
        self.delta_rel = delta_rel
        self.frames = []
        self.z = None       # metres, NaN where the background has no depth
        self.thresh = None

    def add(self, depth):
        self.frames.append(depth[::self.stride, ::self.stride].copy())

    def finalize(self):
        stack = np.stack(self.frames).astype(np.float32)
        stack[stack == 0] = np.nan
        n_valid = np.count_nonzero(~np.isnan(stack), axis=0)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN pixels
            z = np.nanmedian(stack, axis=0) * self.scale
        # No reliable background where most warm-up frames had no depth
        z[2 * n_valid < len(self.frames)] = np.nan
        self.z = z
        # Depth noise grows roughly with z^2, so widen the threshold far away
        self.thresh = np.maximum(self.delta, self.delta_rel * z ** 2)
        self.frames = []

    def foreground(self, depth):
        """Flat bool mask: valid pixels in front of the static background."""
        z = depth[::self.stride, ::self.stride] * self.scale
        no_bg = np.isnan(self.z)
        with np.errstate(invalid="ignore"):
            closer = z < self.z - self.thresh  # False where background is NaN
        # One-sided: pixels now *farther* than the background never count
        return ((z > 0) & (closer | no_bg)).ravel()


def camera_to_world(height, pitch_deg):
    """(R, t) with p_world = R @ p_cam + t.

    camera: x right, y down, z forward, pitched down by pitch_deg.
    world:  x right, y forward, z up, origin on the floor below the camera.
    Columns of R are the camera axes expressed in world.
    """
    s, c = np.sin(np.radians(pitch_deg)), np.cos(np.radians(pitch_deg))
    right = [1.0, 0.0, 0.0]
    down = [0.0, -s, -c]
    forward = [0.0, c, -s]
    return np.column_stack((right, down, forward)), np.array([0.0, 0.0, height])


def crop_box(points, box):
    lo, hi = box
    inside = np.all((points >= lo) & (points <= hi), axis=1)
    return points[inside]


# Why a frame produced no association (Tracker.diag["reason"]):
#   ok               a cluster was associated
#   ok_sparse        track active, no cluster, but >= --sparse-min-points box
#                    points within --sparse-radius of the prediction: used as
#                    the measurement with --sparse-noise x measurement std
#   no_points        no foreground points inside the crop box
#   too_few_points   fewer than --min-points points in the box (DBSCAN not run)
#   dbscan_noise     points exist but DBSCAN formed no cluster (sparser than
#                    --min-samples within --eps); check near_pts: if > 0 the
#                    drone was probably there but too sparse
#   few_points       the relevant cluster has fewer than --min-points points
#   too_small        relevant cluster extent below --min-size
#   too_big          relevant cluster extent above --max-size
#   outside_gate     relevant cluster is drone-like but cand_dist > gate
#   ambiguous        no track and two comparable clusters (--ambiguity-ratio)
#   lost_timeout     coasted longer than --max-coast, track dropped
# "Relevant cluster" = the cluster nearest the prediction (or previous
# candidate) when there is one, else the largest cluster.
REASONS = ("ok", "ok_sparse", "no_points", "too_few_points", "dbscan_noise", "few_points",
           "too_small", "too_big", "outside_gate", "ambiguous", "lost_timeout")


def cluster_verdict(n, extent, dist, args):
    """First check a cluster fails, or 'ok'. dist is None without a reference."""
    if n < args.min_points:
        return "few_points"
    if extent < args.min_size:
        return "too_small"
    if extent > args.max_size:
        return "too_big"
    if dist is not None and dist > args.gate:
        return "outside_gate"
    return "ok"


def select_candidate(points, ref, args):
    """Pick the drone candidate among DBSCAN clusters.

    ref: where the drone is expected (camera frame), or None if unknown.
    Returns (cluster points or None, status, number of valid clusters, diag).
    diag explains the decision: reason (see REASONS), near_pts / near_dist
    (box points within --gate of ref / distance of the closest one), and
    cand_* for the relevant cluster, plus every cluster with its verdict.
    """
    diag = {"reason": "ok", "near_pts": None, "near_dist": None, "cand_n": None,
            "cand_extent": None, "cand_dist": None, "clusters": []}
    if ref is not None and len(points):
        d = np.linalg.norm(points - ref, axis=1)
        diag["near_pts"], diag["near_dist"] = int((d <= args.gate).sum()), float(d.min())
    if len(points) == 0:
        diag["reason"] = "no_points"
        return None, "none", 0, diag
    if len(points) < args.min_points:
        diag["reason"] = "too_few_points"
        return None, "none", 0, diag
    labels = DBSCAN(eps=args.eps, min_samples=args.min_samples).fit_predict(points)
    ids, counts = np.unique(labels[labels >= 0], return_counts=True)
    if len(ids) == 0:
        diag["reason"] = "dbscan_noise"
        return None, "none", 0, diag

    clusters = []  # every DBSCAN cluster, with its measurements and verdict
    for lab, n in zip(ids, counts):
        c = points[labels == lab]
        centroid = c.mean(axis=0)
        extent = float((c.max(axis=0) - c.min(axis=0)).max())
        dist = None if ref is None else float(np.linalg.norm(centroid - ref))
        clusters.append({"points": c, "centroid": centroid, "n": int(n), "extent": extent,
                         "dist": dist, "verdict": cluster_verdict(n, extent, dist, args)})
    diag["clusters"] = clusters

    def explain(c, reason=None):
        diag.update(reason=reason or c["verdict"], cand_n=c["n"],
                    cand_extent=c["extent"], cand_dist=c["dist"])

    if args.no_bg:  # legacy: largest cluster wins
        best = max(clusters, key=lambda c: c["n"])
        if best["n"] >= args.min_points:
            explain(best, "ok")
            return best["points"], "candidate", len(clusters), diag
        explain(best, "few_points")
        return None, "none", len(clusters), diag

    if ref is not None:
        # Reference = track prediction or previous candidate (camera frame):
        # nearest acceptable cluster; otherwise explain the nearest one overall
        valid = [c for c in clusters if c["verdict"] == "ok"]
        n_drone_like = sum(c["verdict"] in ("ok", "outside_gate") for c in clusters)
        if valid:
            best = min(valid, key=lambda c: c["dist"])
            explain(best)
            return best["points"], "candidate", n_drone_like, diag
        explain(min(clusters, key=lambda c: c["dist"]))
        return None, "none", n_drone_like, diag

    valid = sorted((c for c in clusters if c["verdict"] == "ok"),
                   key=lambda c: c["n"], reverse=True)
    if not valid:
        explain(max(clusters, key=lambda c: c["n"]))
        return None, "none", 0, diag
    if len(valid) > 1 and valid[1]["n"] >= args.ambiguity_ratio * valid[0]["n"]:
        explain(valid[0], "ambiguous")
        return None, "ambiguous", len(valid), diag
    explain(valid[0])
    return valid[0]["points"], "candidate", len(valid), diag


class ConstantVelocityKF:
    """Kalman filter on [x, y, z, vx, vy, vz] with white-noise acceleration."""

    def __init__(self, accel_std, meas_std):
        self.q = accel_std ** 2
        self.R = np.eye(3) * meas_std ** 2
        self.H = np.hstack((np.eye(3), np.zeros((3, 3))))
        self.x = None
        self.P = None

    def reset(self):
        self.x = self.P = None

    def predict(self, dt):
        if self.x is None:
            return
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        # Discrete white-noise acceleration model, per axis
        q = self.q * np.array([[dt ** 4 / 4, dt ** 3 / 2],
                               [dt ** 3 / 2, dt ** 2]])
        Q = np.kron(q, np.eye(3))
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z, noise_scale=1.0):
        """noise_scale multiplies the measurement std (less trusted measurement)."""
        R = self.R * noise_scale ** 2
        if self.x is None:
            self.x = np.concatenate((z, np.zeros(3)))
            self.P = np.diag([*np.diag(R), 1.0, 1.0, 1.0])
            return
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P


class Tracker:
    """Candidate confirmation plus a coasting constant-velocity KF track.

    Points and centroids are in the camera frame; the filter runs in world.
    """

    def __init__(self, args, cam_R, cam_t):
        self.args = args
        self.R, self.t = cam_R, cam_t
        self.kf = ConstantVelocityKF(args.accel_std, args.meas_std)
        self.streak, self.prev = 0, None  # tentative candidate (no track yet)
        self.prev_t = self.last_update_t = None
        self.pred = None  # world-frame prediction this frame (before update), if tracking
        self.n_tracks = 0  # tracks confirmed so far
        self.diag = {}  # why this frame was / wasn't associated (see REASONS)
        # Output blending: the filter itself stays exact; the *output* position
        # absorbs each correction and eases into it with time constant blend_tau,
        # so re-acquisition after coasting doesn't jump. Live-compatible.
        self.blend_tau = max(getattr(args, "blend_ms", 0.0), 0.0) / 1000.0
        self.offset = np.zeros(3)

    def _decay(self, dt):
        if self.blend_tau > 0:
            return np.exp(-dt / self.blend_tau)
        return 0.0

    def output(self, ahead=0.0):
        """Smoothed world position `ahead` seconds after the last frame
        (constant-velocity prediction plus the decaying blend offset), or None."""
        if self.kf.x is None:
            return None
        return self.kf.x[:3] + self.kf.x[3:] * ahead + self.offset * self._decay(ahead)

    def to_world(self, p):
        return self.R @ p + self.t

    def to_camera(self, p):
        return self.R.T @ (p - self.t)

    def step(self, t, points):
        """One frame. Returns (cluster, status, n_clusters, centroid_cam,
        world_pos, detected); centroid_cam / world_pos are None without a cluster."""
        a = self.args
        dt = 0.0 if self.prev_t is None else t - self.prev_t
        self.prev_t = t
        self.kf.predict(dt)
        self.offset = self.offset * self._decay(dt)

        # A live filter means a confirmed track: search around its prediction
        tracking = self.kf.x is not None
        self.pred = self.kf.x[:3].copy() if tracking else None
        ref = self.to_camera(self.pred) if tracking else self.prev
        cluster, status, n_clusters, self.diag = select_candidate(points, ref, a)
        self.diag["gate"] = a.gate
        self.diag["since_update"] = (t - self.last_update_t) if tracking else None
        sparse = False
        radius = getattr(a, "sparse_radius", 0.0)
        if (tracking and cluster is None and radius > 0 and len(points)
                and self.diag["reason"] in ("too_few_points", "dbscan_noise", "few_points")):
            # Track active, no cluster, but a few points right at the prediction:
            # too sparse for DBSCAN (partial depth), still almost surely the drone
            near = points[np.linalg.norm(points - ref, axis=1) <= radius]
            if len(near) >= a.sparse_min_points:
                cluster, status, sparse = near, "candidate", True

        centroid = world_pos = None
        detected = False
        if cluster is None:
            if tracking:
                status = "coasting"
            else:
                self.streak, self.prev = 0, None
        else:
            centroid = cluster.mean(axis=0)
            world_pos = self.to_world(centroid)
            if tracking or (self.prev is not None
                            and np.linalg.norm(centroid - self.prev) <= a.gate):
                self.streak += 1
            else:
                self.streak = 1
            self.prev = centroid
            # Confirmed track: a cluster at the prediction is the drone, no re-persist
            detected = tracking or self.streak >= a.persist

        if detected:
            status = "detected"
            if not tracking:
                self.n_tracks += 1
                self.offset = np.zeros(3)
                self.kf.update(world_pos)
            else:
                before = self.kf.x[:3].copy()
                self.kf.update(world_pos, a.sparse_noise if sparse else 1.0)
                if self.blend_tau > 0:
                    self.offset = self.offset + before - self.kf.x[:3]
            self.last_update_t = t
            if sparse:
                self.diag.update(cand_n=len(cluster), cand_dist=float(
                    np.linalg.norm(centroid - ref)), cand_extent=float(
                    (cluster.max(axis=0) - cluster.min(axis=0)).max()))
            self.diag["reason"] = "ok_sparse" if sparse else "ok"
        elif tracking and t - self.last_update_t > a.max_coast:
            status = "lost"
            self.diag["reason"] = "lost_timeout"
            self.offset = np.zeros(3)
            self.kf.reset()
            self.last_update_t = None
            self.streak, self.prev = 0, None
        return cluster, status, n_clusters, centroid, world_pos, detected


class Progress:
    """One-line progress bar + running stats on stderr (no extra dependency).

    On a terminal the line redraws in place every `every` s; when output is
    redirected it prints a plain line every 5 s instead.
    """

    def __init__(self, total, every=0.2, width=28):
        self.total, self.width = total, width
        self.tty = sys.stderr.isatty()
        self.every = every if self.tty else 5.0
        self.t0 = self.last = time.perf_counter()
        self.n = 0
        self.status = collections.Counter()
        self.statuses = []                 # per frame, for gap statistics
        self.t_read = self.t_detect = self.t_log = 0.0  # seconds per stage

    def update(self, status, t_read, t_detect, t_log):
        self.n += 1
        self.t_read += t_read
        self.status[status] += 1
        self.statuses.append(status)
        self.t_detect += t_detect
        self.t_log += t_log
        now = time.perf_counter()
        if now - self.last < self.every:
            return
        self.last = now
        fps = self.n / (now - self.t0)
        if self.total:
            frac = min(self.n / self.total, 1.0)
            fill = int(frac * self.width)
            eta = (self.total - self.n) / fps
            head = (f"[{'#' * fill}{'-' * (self.width - fill)}] "
                    f"{self.n}/{self.total} {100 * frac:3.0f}%  ETA {eta:4.0f}s")
        else:
            head = f"{self.n} frames"
        tracked = self.n - self.status["warmup"]
        line = (f"{head}  {fps:5.1f} fps  "
                f"detected {100 * self.status['detected'] / max(tracked, 1):4.1f}%  "
                f"coasting {self.status['coasting']}  [{status}]")
        if self.tty:
            sys.stderr.write("\r" + line + "\033[K")
        else:
            sys.stderr.write(line + "\n")
        sys.stderr.flush()

    def close(self):
        if self.tty and self.n:
            sys.stderr.write("\n")
            sys.stderr.flush()

    def longest_run(self, status):
        best = cur = 0
        for s in self.statuses:
            cur = cur + 1 if s == status else 0
            best = max(best, cur)
        return best

    def runs(self, status):
        return sum(1 for i, s in enumerate(self.statuses)
                   if s == status and (i == 0 or self.statuses[i - 1] != status))


def print_summary(prog, tracker, sensor_times, detections, speeds, meta):
    elapsed = time.perf_counter() - prog.t0
    n = prog.n
    span = sensor_times[-1] - sensor_times[0] if len(sensor_times) > 1 else 0.0
    rec_fps = (len(sensor_times) - 1) / span if span > 0 else 0.0
    proc_fps = n / elapsed if elapsed > 0 else 0.0
    print("\n== Run ==")
    print(f"  frames         {n}  ({span:.1f} s of recording, {rec_fps:.1f} fps recorded)")
    dropped = meta.get("frames_dropped", {}).get("depth")
    if dropped is not None:
        print(f"  dropped        {dropped} depth frames while recording")
    print(f"  processing     {elapsed:.1f} s, {proc_fps:.1f} fps "
          f"({proc_fps / rec_fps:.2f}x real time)" if rec_fps else
          f"  processing     {elapsed:.1f} s, {proc_fps:.1f} fps")
    print(f"  per frame      {1000 * elapsed / max(n, 1):.1f} ms  "
          f"(read+decode {1000 * prog.t_read / max(n, 1):.1f} ms, "
          f"detect+track {1000 * prog.t_detect / max(n, 1):.1f} ms, "
          f"rerun logging {1000 * prog.t_log / max(n, 1):.1f} ms)")

    tracked = n - prog.status["warmup"]
    dt = span / max(len(sensor_times) - 1, 1)
    print("== Tracking ==")
    for s in ("warmup", "detected", "coasting", "candidate", "ambiguous", "none", "lost"):
        if prog.status[s]:
            base = n if s == "warmup" else max(tracked, 1)
            print(f"  {s:<14} {prog.status[s]:5d}  ({100 * prog.status[s] / base:5.1f}%)")
    print(f"  tracks         {tracker.n_tracks} confirmed, {prog.status['lost']} lost "
          f"after coasting > max-coast")
    if prog.status["coasting"]:
        longest = prog.longest_run("coasting")
        print(f"  coasting gaps  {prog.runs('coasting')}, longest {longest} frames "
              f"({longest * dt:.2f} s)")
    if detections:
        d = np.array(detections)
        print("== Drone (world: x right, y forward, z up) ==")
        for axis, i in (("x", 0), ("y", 1), ("z", 2)):
            print(f"  {axis}  {d[:, i].min():+.2f} .. {d[:, i].max():+.2f} m")
    if speeds:
        s = np.array(speeds)
        print(f"  speed  mean {s.mean():.2f}  p95 {np.percentile(s, 95):.2f}  "
              f"max {s.max():.2f} m/s (filtered, on detected frames)")


def make_blueprint(trail_seconds):
    """Viewer layout. Trails are drawn by Rerun at playback time: each frame logs
    one point / segment, and the view shows only the last trail_seconds of them."""
    window = rrb.VisibleTimeRanges([
        rrb.VisibleTimeRange(
            timeline, start=rrb.TimeRangeBoundary.cursor_relative(seconds=-trail_seconds),
            end=rrb.TimeRangeBoundary.cursor_relative())
        for timeline in ("sensor_time", "wall_time")])
    overrides = {
        "world/trail/raw": [window, rr.Points3D.from_fields().visualizer()],
        "world/trail/filtered": [window, rr.LineStrips3D.from_fields().visualizer()],
        "world/trail/output": [window, rr.LineStrips3D.from_fields().visualizer()],
        # Whole flight: available, but hidden until switched on in the viewer
        "world/trajectory": [rrb.EntityBehavior(visible=False),
                             rr.LineStrips3D.from_fields().visualizer()],
    }
    return rrb.Blueprint(rrb.Horizontal(
        rrb.Spatial3DView(name="world (z up)", origin="world", overrides=overrides),
        rrb.Vertical(
            rrb.Spatial2DView(name="depth", origin="camera/depth"),
            rrb.Spatial2DView(name="foreground", origin="camera/foreground"),
            rrb.Tabs(
                rrb.TimeSeriesView(name="position (world)", origin="plots/position"),
                rrb.TimeSeriesView(name="velocity (world)", origin="plots/velocity"),
                rrb.TimeSeriesView(name="association", origin="plots/association")),
            rrb.TextLogView(name="association log", origin="log")),
        column_shares=[3, 2]))


def detect(rrd_path, args):
    rrd_path = Path(rrd_path)
    meta = json.loads(rrd_path.with_suffix(".json").read_text())
    project = BackProjector(meta["depth_intrinsics"], meta["depth_scale_m_per_unit"],
                            args.stride)
    box = (np.array([args.x_min, args.y_min, args.z_min]),
           np.array([args.x_max, args.y_max, args.z_max]))

    cam_R, cam_t = camera_to_world(args.cam_height, args.cam_pitch)

    csv_path = rrd_path.with_name(rrd_path.stem + "_detections.csv")
    out_rrd = rrd_path.with_name(rrd_path.stem + "_detections.rrd")
    log = args.viewer or not args.no_rrd
    if log:
        blueprint = make_blueprint(args.trail_seconds)
        rr.init("drone_detect", recording_id=rrd_path.stem, spawn=args.viewer,
                default_blueprint=blueprint)
        if not args.no_rrd:
            rr.save(out_rrd, default_blueprint=blueprint)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        # Camera-frame entities live under world/camera; Rerun applies the pose
        rr.log("world/camera", rr.Transform3D(translation=cam_t, mat3x3=cam_R),
               static=True)
        rr.log("world/camera/axes", rr.Arrows3D(
            origins=[[0, 0, 0]] * 3, vectors=np.eye(3) * 0.2,
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            labels=["cam x (right)", "cam y (down)", "cam z (depth)"]), static=True)
        lo, hi = box
        rr.log("world/camera/crop_box", rr.Boxes3D(centers=[(lo + hi) / 2],
                                                    sizes=[hi - lo]), static=True)

    bg = None if args.no_bg else DepthBackground(
        meta["depth_scale_m_per_unit"], args.stride, args.bg_delta, args.bg_delta_rel)

    tracker = Tracker(args, cam_R, cam_t)
    kf = tracker.kf
    n_frames = 0
    detections = []
    strips = []  # trajectory, split into segments wherever detection drops out
    last_detected = None
    last_filtered = None  # filtered position last frame, for trail segments
    total = meta.get("frames_logged", {}).get("depth")
    if args.max_frames:
        total = min(total, args.max_frames) if total else args.max_frames
    prog = Progress(total)
    reasons = collections.Counter()
    pred_path = rrd_path.with_name(rrd_path.stem + "_predictions.csv")
    pred_file = open(pred_path, "w", newline="") if args.predict_hz > 0 else None
    pred_writer = csv.DictWriter(pred_file, fieldnames=PRED_FIELDS) if pred_file else None
    if pred_writer:
        pred_writer.writeheader()
    last_output = None  # last output position logged, for the output trail
    prev_sensor = None
    t_end = time.perf_counter()  # end of the previous frame, to time reading
    sensor_times, speeds = [], []
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for frame, (wall, sensor, depth) in enumerate(iter_depth_frames(rrd_path)):
            if args.max_frames and frame >= args.max_frames:
                break
            t0 = time.perf_counter()
            t_read = t0 - t_end
            n_frames += 1
            sensor_times.append(sensor)
            row = {"frame": frame, "wall_time": f"{wall:.6f}",
                   "sensor_time": f"{sensor:.6f}", "detected": 0}
            cluster, fg = None, None
            cropped = np.empty((0, 3))

            if bg is not None and frame < args.bg_frames:
                # Warm-up: learn the static scene, no detection, no filter
                bg.add(depth)
                row["status"] = "warmup"
                if frame == args.bg_frames - 1:
                    bg.finalize()
                    if log:
                        z = bg.z.ravel()
                        rr.log("world/camera/background", rr.Points3D(
                            project.points(z, ~np.isnan(z)), radii=0.004,
                            colors=[90, 90, 90]), static=True)
                writer.writerow(row)
                t1 = time.perf_counter()
                if log:
                    rr.set_time("sensor_time", duration=sensor)
                    rr.set_time("wall_time", timestamp=wall)
                    rr.log("camera/depth", rr.DepthImage(depth,
                           meter=1.0 / meta["depth_scale_m_per_unit"]))
                t_end = time.perf_counter()
                prog.update("warmup", t_read, t1 - t0, t_end - t1)
                continue

            if bg is not None:
                fg = bg.foreground(depth)
                row["n_foreground"] = int(fg.sum())
            cropped = crop_box(project(depth, fg), box)
            if args.predict_hz > 0 and kf.x is not None and prev_sensor is not None:
                # Predictions between depth frames, from the last filter state
                # (what a live system would output on its own clock)
                step = 1.0 / args.predict_hz
                for ts in np.arange(prev_sensor + step, sensor - 1e-6, step):
                    out = tracker.output(ts - prev_sensor)
                    vel_now = kf.x[3:]
                    if pred_writer:
                        pred_writer.writerow({"sensor_time": f"{ts:.6f}", "kind": "predict",
                                              **{k: f"{v:.4f}" for k, v in zip(
                                                  ("x", "y", "z", "vx", "vy", "vz"),
                                                  (*out, *vel_now))}})
                    if log:
                        rr.set_time("sensor_time", duration=ts)
                        rr.set_time("wall_time", timestamp=prev_wall + ts - prev_sensor)
                        rr.log("world/output", rr.Points3D([out], radii=0.02,
                                                            colors=[255, 255, 255]))
                        if last_output is not None:
                            rr.log("world/trail/output", rr.LineStrips3D(
                                [[last_output, out]], radii=0.003, colors=[255, 255, 255]))
                        for axis, v in zip("xyz", out):
                            rr.log(f"plots/position/{axis}_output", rr.Scalars(v))
                    last_output = out
            prev_sensor, prev_wall = sensor, wall

            cluster, status, n_clusters, centroid, world_pos, detected = \
                tracker.step(sensor, cropped)
            diag = tracker.diag
            reasons[diag["reason"]] += 1
            row["reason"] = diag["reason"]
            for k in ("since_update", "gate", "near_dist", "cand_extent", "cand_dist"):
                if diag.get(k) is not None:
                    row[k] = f"{diag[k]:.4f}"
            for k in ("near_pts", "cand_n"):
                if diag.get(k) is not None:
                    row[k] = diag[k]
            row.update({"status": status, "n_cropped": len(cropped),
                        "n_clusters": n_clusters, "detected": int(detected)})

            if cluster is not None:
                size = cluster.max(axis=0) - cluster.min(axis=0)
                row.update({k: f"{v:.4f}" for k, v in zip(
                    ("x", "y", "z", "cam_x", "cam_y", "cam_z",
                     "size_x", "size_y", "size_z"), (*world_pos, *centroid, *size))})
                row["n_points"] = len(cluster)
            if detected:
                detections.append(world_pos)
                if last_detected != frame - 1:
                    strips.append([])
                strips[-1].append(world_pos)
                last_detected = frame

            if kf.x is not None:
                pos, vel = kf.x[:3], kf.x[3:]
                row.update({k: f"{v:.4f}" for k, v in zip(
                    ("fx", "fy", "fz", "vx", "vy", "vz", "speed"),
                    (*pos, *vel, np.linalg.norm(vel)))})
                if detected:
                    speeds.append(float(np.linalg.norm(vel)))
                out = tracker.output()
                row.update({k: f"{v:.4f}" for k, v in zip(("ox", "oy", "oz"), out)})
                if pred_writer:
                    pred_writer.writerow({"sensor_time": f"{sensor:.6f}",
                                          "kind": "update" if detected else "coast",
                                          **{k: f"{v:.4f}" for k, v in zip(
                                              ("x", "y", "z", "vx", "vy", "vz"),
                                              (*out, *vel))}})
            writer.writerow(row)
            t1 = time.perf_counter()

            if log:
                rr.set_time("sensor_time", duration=sensor)
                rr.set_time("wall_time", timestamp=wall)
                rr.log("camera/depth", rr.DepthImage(depth,
                       meter=1.0 / meta["depth_scale_m_per_unit"]))
                if fg is not None:
                    rr.log("camera/foreground", rr.Image(
                        fg.reshape(bg.z.shape).astype(np.uint8) * 255))
                rr.log("world/camera/cropped", rr.Points3D(
                    cropped, radii=0.005, colors=[150, 150, 150]))
                if cluster is not None:
                    color = [0, 220, 0] if row["detected"] else [255, 160, 0]
                    rr.log("world/camera/cluster", rr.Points3D(
                        cluster, radii=0.006, colors=color))
                else:
                    rr.log("world/camera/cluster", rr.Clear(recursive=False))
                # Every DBSCAN cluster, coloured by verdict, labelled with the numbers
                cl = diag["clusters"]
                if cl:
                    rr.log("world/camera/clusters", rr.Points3D(
                        np.concatenate([c["points"] for c in cl]), radii=0.004,
                        colors=np.concatenate([[VERDICT_COLORS[c["verdict"]]] * c["n"]
                                               for c in cl])))
                    rr.log("world/camera/clusters/labels", rr.Points3D(
                        [c["centroid"] for c in cl], radii=0.01,
                        colors=[VERDICT_COLORS[c["verdict"]] for c in cl],
                        labels=[f"{c['verdict']} n={c['n']} ext={c['extent']:.2f}"
                                + ("" if c["dist"] is None else f" d={c['dist']:.2f}")
                                for c in cl], show_labels=True))
                else:
                    rr.log("world/camera/clusters", rr.Clear(recursive=True))
                for k in ("cand_dist", "near_dist", "since_update"):
                    if diag.get(k) is not None:
                        rr.log(f"plots/association/{k}", rr.Scalars(diag[k]))
                if diag.get("since_update") is not None:
                    rr.log("plots/association/gate", rr.Scalars(diag["gate"]))
                    rr.log("plots/association/max_coast", rr.Scalars(args.max_coast))
                if diag.get("near_pts") is not None:
                    rr.log("plots/association/near_pts", rr.Scalars(diag["near_pts"]))
                if diag["reason"] != "ok" or status != "detected":
                    parts = [f"frame {frame} {status}: {diag['reason']}"]
                    for k, fmt in (("cand_dist", "d={:.3f}"), ("gate", "gate={:.2f}"),
                                   ("cand_n", "n={}"), ("cand_extent", "ext={:.3f}"),
                                   ("near_pts", "pts_in_gate={}"),
                                   ("near_dist", "closest_pt={:.3f}"),
                                   ("since_update", "since_update={:.2f}s")):
                        if diag.get(k) is not None:
                            parts.append(fmt.format(diag[k]))
                    level = "WARN" if status in ("coasting", "lost") else "INFO"
                    rr.log("log/association", rr.TextLog(" ".join(parts), level=level))
                if kf.x is not None:
                    out = tracker.output()
                    rr.log("world/output", rr.Points3D([out], radii=0.02,
                                                        colors=[255, 255, 255]))
                    if last_output is not None:
                        rr.log("world/trail/output", rr.LineStrips3D(
                            [[last_output, out]], radii=0.003, colors=[255, 255, 255]))
                    for axis, v in zip("xyz", out):
                        rr.log(f"plots/position/{axis}_output", rr.Scalars(v))
                    last_output = out
                else:
                    rr.log("world/output", rr.Clear(recursive=False))
                    last_output = None
                if row["detected"]:
                    rr.log("world/drone", rr.Points3D([world_pos], radii=0.03,
                                                       colors=[255, 0, 0]))
                else:
                    rr.log("world/drone", rr.Clear(recursive=False))
                if row["detected"]:
                    for axis, v in zip("xyz", world_pos):
                        rr.log(f"plots/position/{axis}_raw", rr.Scalars(v))
                    # Raw detections: one dot per frame, windowed by the blueprint
                    rr.log("world/trail/raw", rr.Points3D(
                        [world_pos], radii=0.008, colors=[255, 0, 0]))
                if tracker.pred is not None:
                    # Where the tracker searched this frame: prediction + gate
                    rr.log("world/prediction", rr.Ellipsoids3D(
                        centers=[tracker.pred], half_sizes=[[args.gate] * 3],
                        colors=[255, 0, 255], line_radii=0.001,
                        fill_mode=rr.components.FillMode.MajorWireframe))
                else:
                    rr.log("world/prediction", rr.Clear(recursive=False))
                if kf.x is not None:
                    if last_filtered is not None:
                        # Blue: updated by a detection. Magenta: prediction only
                        color = [0, 120, 255] if row["detected"] else [255, 0, 255]
                        rr.log("world/trail/filtered", rr.LineStrips3D(
                            [[last_filtered, pos]], radii=0.004, colors=[color]))
                    last_filtered = pos.copy()
                else:
                    last_filtered = None
                if kf.x is not None:
                    rr.log("world/filtered", rr.Points3D([pos], radii=0.025,
                                                          colors=[0, 120, 255]))
                    rr.log("world/filtered/velocity", rr.Arrows3D(
                        origins=[pos], vectors=[vel * args.arrow_scale],
                        colors=[0, 120, 255]))
                    for axis, p_, v in zip("xyz", pos, vel):
                        rr.log(f"plots/position/{axis}", rr.Scalars(p_))
                        rr.log(f"plots/velocity/v{axis}", rr.Scalars(v))
                    rr.log("plots/velocity/speed", rr.Scalars(np.linalg.norm(vel)))
                else:
                    rr.log("world/filtered", rr.Clear(recursive=True))
            t_end = time.perf_counter()
            prog.update(row["status"], t_read, t1 - t0, t_end - t1)
    prog.close()

    if log and strips:
        # Whole flight at once, independent of the time cursor.
        rr.log("world/trajectory", rr.LineStrips3D(
            strips, radii=0.002, colors=[255, 200, 200]), static=True)

    if pred_file:
        pred_file.close()
    print_summary(prog, tracker, sensor_times, detections, speeds, meta)
    if reasons["ok_sparse"]:
        print(f"== Sparse associations (few points at the prediction): {reasons['ok_sparse']} ==")
    print("== Why frames were not associated (after warm-up) ==")
    for r in REASONS:
        if r not in ("ok", "ok_sparse") and reasons[r]:
            print(f"  {r:<15} {reasons[r]:5d}")
    print("== Output ==")
    print(f"Wrote {csv_path}")
    if pred_file:
        print(f"Wrote {pred_path}  ({args.predict_hz:g} Hz predictions + frame updates)")
    if not args.no_rrd:
        print(f"Wrote {out_rrd} -- view with: rerun {out_rrd}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", help=".rrd from realsense_record.py "
                                          "(its .json sidecar must be next to it)")
    box = parser.add_argument_group("crop box, camera frame "
                                    "(x right, y down, z forward), metres")
    box.add_argument("--x-min", type=float, default=-1.0)
    box.add_argument("--x-max", type=float, default=1.0)
    box.add_argument("--y-min", type=float, default=-1.0)
    box.add_argument("--y-max", type=float, default=1.0)
    box.add_argument("--z-min", type=float, default=0.3)
    box.add_argument("--z-max", type=float, default=2.0)
    bgp = parser.add_argument_group("static background (drone out of view at start)")
    bgp.add_argument("--bg-frames", type=int, default=30,
                     help="first n depth frames used to learn the background")
    bgp.add_argument("--bg-delta", type=float, default=0.10,
                     help="min distance in front of the background to be foreground, metres")
    bgp.add_argument("--bg-delta-rel", type=float, default=0.02,
                     help="extra threshold = this * background_depth^2 (noise grows "
                          "with depth); 0 disables")
    bgp.add_argument("--no-bg", action="store_true",
                     help="old behaviour: no background subtraction, largest cluster wins")
    pose = parser.add_argument_group("camera pose -> world frame (x right, y forward, z up)")
    pose.add_argument("--cam-height", type=float, default=0.0,
                      help="camera lens height above the floor, metres "
                           "(0 = world z is height relative to the camera)")
    pose.add_argument("--cam-pitch", type=float, default=0.0,
                      help="camera tilt, degrees; positive = looking down")
    det = parser.add_argument_group("detection")
    det.add_argument("--stride", type=int, default=2,
                     help="use every n-th pixel in each direction")
    det.add_argument("--eps", type=float, default=0.05,
                     help="DBSCAN neighbourhood radius, metres")
    det.add_argument("--min-samples", type=int, default=5,
                     help="DBSCAN core-point neighbour count")
    det.add_argument("--min-points", type=int, default=1,
                     help="smallest cluster accepted as a candidate")
    det.add_argument("--persist", type=int, default=2,
                     help="consecutive frames before a candidate counts as the drone")
    det.add_argument("--gate", type=float, default=0.5,
                     help="max centroid jump between consecutive frames, metres")
    det.add_argument("--min-size", type=float, default=0.0,
                     help="smallest drone-like cluster extent (largest axis), metres; "
                          "0 = off (far away the drone is only ~10 points, ~2 cm)")
    det.add_argument("--max-size", type=float, default=0.4,
                     help="largest drone-like cluster extent (largest axis), metres")
    det.add_argument("--ambiguity-ratio", type=float, default=0.5,
                     help="with no track, reject if the 2nd cluster has at least "
                          "this fraction of the largest one's points")
    filt = parser.add_argument_group("filter")
    filt.add_argument("--accel-std", type=float, default=3.0,
                      help="process noise: expected acceleration std, m/s^2")
    filt.add_argument("--meas-std", type=float, default=0.02,
                      help="measurement noise: detection position std, metres")
    filt.add_argument("--max-coast", type=float, default=1.5,
                      help="reset the filter after this many seconds without a detection")
    filt.add_argument("--arrow-scale", type=float, default=0.3,
                      help="velocity arrow length in the viewer = velocity * this (s)")
    det.add_argument("--sparse-radius", type=float, default=0.10,
                     help="while tracking, if no cluster forms, accept box points within "
                          "this distance of the prediction as the drone, metres; 0 = off")
    det.add_argument("--sparse-min-points", type=int, default=2,
                     help="minimum points for such a sparse association")
    det.add_argument("--sparse-noise", type=float, default=3.0,
                     help="sparse associations are trusted less: measurement std x this")
    filt.add_argument("--blend-ms", type=float, default=100.0,
                      help="output smoothing: ease the output position into each filter "
                           "correction with this time constant (filter stays exact); 0 = off")
    filt.add_argument("--predict-hz", type=float, default=90.0,
                      help="also output constant-velocity predictions at this rate between "
                           "depth frames (Rerun world/output + _predictions.csv); 0 = off")
    parser.add_argument("--trail-seconds", type=float, default=3.0,
                        help="trail length shown in the viewer (default layout); "
                             "change it live in the view's 'Visible time range'")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="stop after this many frames (0 = all)")
    parser.add_argument("--viewer", action="store_true",
                        help="spawn a live Rerun viewer while processing")
    parser.add_argument("--no-rrd", action="store_true",
                        help="don't write the _detections.rrd visualization")
    args = parser.parse_args()
    detect(args.recording, args)
