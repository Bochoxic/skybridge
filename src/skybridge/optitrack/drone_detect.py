# -*- coding: utf-8 -*-
"""
Offline drone detection from a RealSense depth recording made with
realsense_record.py, plus a constant-velocity Kalman filter that turns the
detections into smoothed position and velocity.

The algorithm itself lives in skybridge.perception and is the same code that
runs live; this script replays a recording through it and writes the results.

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
REASONS in skybridge/perception/detector.py), and plots/association shows
cand_dist vs gate, the closest box point to the prediction, points within the
gate and time since the last update vs --max-coast.
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
from dataclasses import fields
from pathlib import Path

import numpy as np
import rerun as rr

from skybridge.perception import CameraModel, DepthFrame
from skybridge.perception.config import PipelineParams
from skybridge.perception.detector import REASONS
from skybridge.perception.pipeline import DepthPipeline
from skybridge.perception.sources import iter_depth_frames
from skybridge.perception.viz import RerunSink, make_blueprint

CSV_FIELDS = ["frame", "wall_time", "sensor_time", "status", "n_foreground",
              "n_cropped", "n_clusters", "detected",
              "x", "y", "z", "cam_x", "cam_y", "cam_z",
              "n_points", "size_x", "size_y", "size_z",
              "fx", "fy", "fz", "vx", "vy", "vz", "speed",
              "ox", "oy", "oz",
              "reason", "since_update", "gate", "near_pts", "near_dist",
              "cand_n", "cand_extent", "cand_dist"]
PRED_FIELDS = ["sensor_time", "kind", "x", "y", "z", "vx", "vy", "vz"]


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


def params_from_args(args):
    return PipelineParams(**{f.name: getattr(args, f.name) for f in fields(PipelineParams)})


def result_row(row, res):
    """Fill a CSV row from a tracked (non-warm-up) frame."""
    diag, status = res.diag, res.estimate.status
    row["reason"] = diag["reason"]
    for k in ("since_update", "gate", "near_dist", "cand_extent", "cand_dist"):
        if diag.get(k) is not None:
            row[k] = f"{diag[k]:.4f}"
    for k in ("near_pts", "cand_n"):
        if diag.get(k) is not None:
            row[k] = diag[k]
    row.update({"status": status, "n_cropped": len(res.cropped),
                "n_clusters": res.n_clusters, "detected": int(res.detected)})
    if res.foreground is not None:
        row["n_foreground"] = int(res.foreground.sum())

    if res.cluster is not None:
        size = res.cluster.max(axis=0) - res.cluster.min(axis=0)
        row.update({k: f"{v:.4f}" for k, v in zip(
            ("x", "y", "z", "cam_x", "cam_y", "cam_z",
             "size_x", "size_y", "size_z"), (*res.world_pos, *res.centroid, *size))})
        row["n_points"] = len(res.cluster)
    if res.filtered is not None:
        row.update({k: f"{v:.4f}" for k, v in zip(
            ("fx", "fy", "fz", "vx", "vy", "vz", "speed"),
            (*res.filtered, *res.velocity, np.linalg.norm(res.velocity)))})
        row.update({k: f"{v:.4f}" for k, v in zip(("ox", "oy", "oz"), res.output)})


def detect(rrd_path, args):
    rrd_path = Path(rrd_path)
    meta = json.loads(rrd_path.with_suffix(".json").read_text())
    pipeline = DepthPipeline(params_from_args(args), CameraModel.from_meta(meta))
    tracker = pipeline.tracker
    kf = tracker.kf

    csv_path = rrd_path.with_name(rrd_path.stem + "_detections.csv")
    out_rrd = rrd_path.with_name(rrd_path.stem + "_detections.rrd")
    sink = None
    if args.viewer or not args.no_rrd:
        blueprint = make_blueprint(args.trail_seconds)
        rr.init("drone_detect", recording_id=rrd_path.stem, spawn=args.viewer,
                default_blueprint=blueprint)
        if not args.no_rrd:
            rr.save(out_rrd, default_blueprint=blueprint)
        sink = RerunSink(pipeline, arrow_scale=args.arrow_scale)
        sink.log_static()

    detections = []
    strips = []  # trajectory, split into segments wherever detection drops out
    last_detected = None
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
    prev_sensor = prev_wall = None
    t_end = time.perf_counter()  # end of the previous frame, to time reading
    sensor_times, speeds = [], []
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for index, (wall, sensor, depth) in enumerate(iter_depth_frames(rrd_path)):
            if args.max_frames and index >= args.max_frames:
                break
            t0 = time.perf_counter()
            t_read = t0 - t_end
            sensor_times.append(sensor)
            row = {"frame": index, "wall_time": f"{wall:.6f}",
                   "sensor_time": f"{sensor:.6f}", "detected": 0}

            if args.predict_hz > 0 and kf.x is not None and prev_sensor is not None:
                # Predictions between depth frames, from the last filter state
                # (what a live system would output on its own clock)
                step = 1.0 / args.predict_hz
                for ts in np.arange(prev_sensor + step, sensor - 1e-6, step):
                    out = tracker.output(ts - prev_sensor)
                    if pred_writer:
                        pred_writer.writerow({"sensor_time": f"{ts:.6f}", "kind": "predict",
                                              **{k: f"{v:.4f}" for k, v in zip(
                                                  ("x", "y", "z", "vx", "vy", "vz"),
                                                  (*out, *kf.x[3:]))}})
                    if sink:
                        sink.log_prediction(ts, prev_wall + ts - prev_sensor, out)
            prev_sensor, prev_wall = sensor, wall

            res = pipeline.process(DepthFrame(depth=depth, t=sensor, wall=wall, seq=index))
            status = res.estimate.status
            if res.warmup:
                row["status"] = status
            else:
                reasons[res.diag["reason"]] += 1
                result_row(row, res)
                if res.detected:
                    detections.append(res.world_pos)
                    if last_detected != index - 1:
                        strips.append([])
                    strips[-1].append(res.world_pos)
                    last_detected = index
                if res.filtered is not None:
                    if res.detected:
                        speeds.append(float(np.linalg.norm(res.velocity)))
                    if pred_writer:
                        pred_writer.writerow({"sensor_time": f"{sensor:.6f}",
                                              "kind": "update" if res.detected else "coast",
                                              **{k: f"{v:.4f}" for k, v in zip(
                                                  ("x", "y", "z", "vx", "vy", "vz"),
                                                  (*res.output, *res.velocity))}})
            writer.writerow(row)
            t1 = time.perf_counter()
            if sink:
                sink.log_result(res)
            t_end = time.perf_counter()
            prog.update(status, t_read, t1 - t0, t_end - t1)
    prog.close()

    if sink and strips:
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
    D = PipelineParams()  # the defaults live in one place, shared with the live config
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", help=".rrd from realsense_record.py "
                                          "(its .json sidecar must be next to it)")
    box = parser.add_argument_group("crop box, camera frame "
                                    "(x right, y down, z forward), metres")
    box.add_argument("--x-min", type=float, default=D.x_min)
    box.add_argument("--x-max", type=float, default=D.x_max)
    box.add_argument("--y-min", type=float, default=D.y_min)
    box.add_argument("--y-max", type=float, default=D.y_max)
    box.add_argument("--z-min", type=float, default=D.z_min)
    box.add_argument("--z-max", type=float, default=D.z_max)
    bgp = parser.add_argument_group("static background (drone out of view at start)")
    bgp.add_argument("--bg-frames", type=int, default=D.bg_frames,
                     help="first n depth frames used to learn the background")
    bgp.add_argument("--bg-delta", type=float, default=D.bg_delta,
                     help="min distance in front of the background to be foreground, metres")
    bgp.add_argument("--bg-delta-rel", type=float, default=D.bg_delta_rel,
                     help="extra threshold = this * background_depth^2 (noise grows "
                          "with depth); 0 disables")
    bgp.add_argument("--no-bg", action="store_true",
                     help="old behaviour: no background subtraction, largest cluster wins")
    pose = parser.add_argument_group("camera pose -> world frame (x right, y forward, z up)")
    pose.add_argument("--cam-height", type=float, default=D.cam_height,
                      help="camera lens height above the floor, metres "
                           "(0 = world z is height relative to the camera)")
    pose.add_argument("--cam-pitch", type=float, default=D.cam_pitch,
                      help="camera tilt, degrees; positive = looking down")
    det = parser.add_argument_group("detection")
    det.add_argument("--stride", type=int, default=D.stride,
                     help="use every n-th pixel in each direction")
    det.add_argument("--eps", type=float, default=D.eps,
                     help="DBSCAN neighbourhood radius, metres")
    det.add_argument("--min-samples", type=int, default=D.min_samples,
                     help="DBSCAN core-point neighbour count")
    det.add_argument("--min-points", type=int, default=D.min_points,
                     help="smallest cluster accepted as a candidate")
    det.add_argument("--max-points", type=int, default=D.max_points,
                     help="above this many box points, cluster only those near the track, "
                          "or skip the frame without one; 0 = no limit")
    det.add_argument("--persist", type=int, default=D.persist,
                     help="consecutive frames before a candidate counts as the drone")
    det.add_argument("--gate", type=float, default=D.gate,
                     help="max centroid jump between consecutive frames, metres")
    det.add_argument("--min-size", type=float, default=D.min_size,
                     help="smallest drone-like cluster extent (largest axis), metres; "
                          "0 = off (far away the drone is only ~10 points, ~2 cm)")
    det.add_argument("--max-size", type=float, default=D.max_size,
                     help="largest drone-like cluster extent (largest axis), metres")
    det.add_argument("--ambiguity-ratio", type=float, default=D.ambiguity_ratio,
                     help="with no track, reject if the 2nd cluster has at least "
                          "this fraction of the largest one's points")
    filt = parser.add_argument_group("filter")
    filt.add_argument("--accel-std", type=float, default=D.accel_std,
                      help="process noise: expected acceleration std, m/s^2")
    filt.add_argument("--meas-std", type=float, default=D.meas_std,
                      help="measurement noise: detection position std, metres")
    filt.add_argument("--max-coast", type=float, default=D.max_coast,
                      help="reset the filter after this many seconds without a detection")
    filt.add_argument("--arrow-scale", type=float, default=0.3,
                      help="velocity arrow length in the viewer = velocity * this (s)")
    det.add_argument("--sparse-radius", type=float, default=D.sparse_radius,
                     help="while tracking, if no cluster forms, accept box points within "
                          "this distance of the prediction as the drone, metres; 0 = off")
    det.add_argument("--sparse-min-points", type=int, default=D.sparse_min_points,
                     help="minimum points for such a sparse association")
    det.add_argument("--sparse-noise", type=float, default=D.sparse_noise,
                     help="sparse associations are trusted less: measurement std x this")
    filt.add_argument("--blend-ms", type=float, default=D.blend_ms,
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
