# -*- coding: utf-8 -*-
"""
Record RGB + depth + IMU (accel/gyro) from an Intel RealSense D455, logged
live to Rerun and saved as a .rrd file for fast visualization/scrubbing.
Meant for building a depth-based (later fused with object detection) drone
detector: the RealSense is the sensor that has to find the drone.

Run:
    uv run realsense_record.py --label hover_test --seconds 30 --no-viewer

Each run writes two files next to each other in data/realsense/ (git-ignored):
    2026-10-28_14-32-05_hover_test.rrd    -- the recording
    2026-10-28_14-32-05_hover_test.json   -- camera metadata (see below)

View any recording with:
    rerun data/realsense/2026-10-28_14-32-05_hover_test.rrd

================================================================================
WHAT GETS LOGGED, AND WHERE TO FIND IT IN THE RERUN VIEWER
================================================================================
    /camera/color        -- RGB image stream (JPEG-compressed)
    /camera/depth        -- depth image stream, lossless uint16
    /imu/accel/x,y,z      -- accelerometer, as 3 scalar time series
    /imu/gyro/x,y,z       -- gyroscope, as 3 scalar time series

Two timelines on every entity:
    sensor_time -- seconds since the first frame (hardware timestamps)
    wall_time   -- absolute Unix time (RealSense global time), for aligning
                   with other logs (e.g. flight-controller telemetry)

The JSON sidecar holds what is needed to work with the data offline:
    label, start wall time, stream settings, depth scale (m per unit),
    depth + color intrinsics, depth->color extrinsics, emitter/exposure
    options, and per-stream frame / dropped-frame counts (filled at the end).
================================================================================
"""
import argparse
import json
import queue
import re
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pyrealsense2 as rs
import rerun as rr

DEFAULT_OUT_DIR = Path(__file__).resolve().parents[3] / "data" / "realsense"
JPEG_QUALITY = 90


def make_run_paths(out_dir, label):
    """Timestamped (and optionally labelled) .rrd / .json paths in out_dir."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if label:
        stem += "_" + re.sub(r"[^A-Za-z0-9._-]+", "-", label).strip("-")
    return out_dir / f"{stem}.rrd", out_dir / f"{stem}.json"


def intrinsics_dict(profile):
    i = profile.as_video_stream_profile().get_intrinsics()
    return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy,
            "ppx": i.ppx, "ppy": i.ppy, "model": str(i.model),
            "coeffs": list(i.coeffs)}


def option_or_none(sensor, option):
    return sensor.get_option(option) if sensor.supports(option) else None


def record(rrd_path, meta_path, label, seconds, res=(848, 480), fps=30,
           enable_imu=True, spawn_viewer=True):
    rr.init("realsense_d455", spawn=spawn_viewer)
    rr.save(rrd_path)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, res[0], res[1], rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, res[0], res[1], rs.format.z16, fps)
    if enable_imu:
        # D455 supports accel @ 400/200/100 Hz and gyro @ 400/200 Hz only
        # (independent streams, not sample-aligned with each other or color/depth)
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

    # Callback + queue so every frame is kept: polling wait_for_frames() would
    # return one IMU sample per video frameset and lose the rest.
    frame_q = queue.Queue()
    profile = pipeline.start(config, frame_q.put)
    device = profile.get_device()

    # Global time maps hardware timestamps onto the host clock (Unix epoch, ms),
    # so frames from all streams can be matched to other logs (e.g. flight telemetry).
    for sensor in device.query_sensors():
        if sensor.supports(rs.option.global_time_enabled):
            sensor.set_option(rs.option.global_time_enabled, 1)

    depth_sensor = device.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()  # metres per depth unit
    depth_profile = profile.get_stream(rs.stream.depth)
    color_profile = profile.get_stream(rs.stream.color)
    depth_intr = intrinsics_dict(depth_profile)
    color_intr = intrinsics_dict(color_profile)
    extr = depth_profile.get_extrinsics_to(color_profile)

    meta = {
        "label": label,
        "start_wall_time": time.time(),
        "start_iso": datetime.now().astimezone().isoformat(),
        "device": device.get_info(rs.camera_info.name),
        "serial": device.get_info(rs.camera_info.serial_number),
        "firmware": device.get_info(rs.camera_info.firmware_version),
        "resolution": list(res),
        "fps": fps,
        "imu_enabled": enable_imu,
        "depth_scale_m_per_unit": depth_scale,
        "depth_intrinsics": depth_intr,
        "color_intrinsics": color_intr,
        "depth_to_color_extrinsics": {"rotation": list(extr.rotation),
                                      "translation": list(extr.translation)},
        "depth_options": {
            "emitter_enabled": option_or_none(depth_sensor, rs.option.emitter_enabled),
            "laser_power": option_or_none(depth_sensor, rs.option.laser_power),
            "exposure": option_or_none(depth_sensor, rs.option.exposure),
            "gain": option_or_none(depth_sensor, rs.option.gain),
        },
    }
    meta_path.write_text(json.dumps(meta, indent=2))

    for name, intr in (("depth", depth_intr), ("color", color_intr)):
        rr.log(f"camera/{name}", rr.Pinhole(
            image_from_camera=[[intr["fx"], 0, intr["ppx"]],
                               [0, intr["fy"], intr["ppy"]],
                               [0, 0, 1]],
            resolution=[intr["width"], intr["height"]]), static=True)

    t0 = None
    counts = {s: 0 for s in ("color", "depth", "accel", "gyro")}
    dropped = {s: 0 for s in counts}
    last_num = {}

    def stamp(frame, stream):
        """Set both timelines for this frame and track dropped frames."""
        nonlocal t0
        t = frame.get_timestamp() / 1000.0  # ms -> s
        if t0 is None:
            t0 = t
        rr.set_time("sensor_time", duration=t - t0)
        rr.set_time("wall_time", timestamp=t)
        counts[stream] += 1
        n = frame.get_frame_number()
        if stream in last_num and n > last_num[stream] + 1:
            dropped[stream] += n - last_num[stream] - 1
        last_num[stream] = n

    try:
        print(f"Recording '{label or 'unlabelled'}' for {seconds}s -> {rrd_path}"
              "\nCtrl+C to stop early.")
        start = time.time()
        while time.time() - start < seconds:
            try:
                item = frame_q.get(timeout=1.0)
            except queue.Empty:
                continue
            frames = list(item.as_frameset()) if item.is_frameset() else [item]

            for frame in frames:
                stream = frame.get_profile().stream_type()

                if stream == rs.stream.color:
                    stamp(frame, "color")
                    img_bgr = np.asanyarray(frame.get_data())
                    img_rgb = np.ascontiguousarray(img_bgr[:, :, ::-1])  # BGR -> RGB
                    rr.log("camera/color",
                           rr.Image(img_rgb).compress(jpeg_quality=JPEG_QUALITY))

                elif stream == rs.stream.depth:
                    stamp(frame, "depth")
                    depth_img = np.asanyarray(frame.get_data())  # uint16
                    rr.log("camera/depth",
                           rr.DepthImage(depth_img, meter=1.0 / depth_scale))

                elif stream in (rs.stream.accel, rs.stream.gyro):
                    name = "accel" if stream == rs.stream.accel else "gyro"
                    stamp(frame, name)
                    d = frame.as_motion_frame().get_motion_data()
                    rr.log(f"imu/{name}/x", rr.Scalars(d.x))
                    rr.log(f"imu/{name}/y", rr.Scalars(d.y))
                    rr.log(f"imu/{name}/z", rr.Scalars(d.z))
    except KeyboardInterrupt:
        print("Stopped early by user.")
    finally:
        pipeline.stop()
        meta["duration_s"] = time.time() - start
        meta["frames_logged"] = counts
        meta["frames_dropped"] = dropped
        meta_path.write_text(json.dumps(meta, indent=2))

    print(f"Done. logged={counts} dropped={dropped}")
    print(f"Saved {rrd_path} and {meta_path}\nView with: rerun {rrd_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--label", default="",
                        help="short name for this take, added to the filename "
                             "and stored in the metadata (e.g. hover_test)")
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR),
                        help="directory for recordings (default: data/realsense)")
    parser.add_argument("--seconds", type=float, default=20.0,
                        help="recording duration")
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--no-imu", action="store_true",
                        help="disable IMU recording (color+depth only)")
    parser.add_argument("--no-viewer", action="store_true",
                        help="don't spawn a live Rerun viewer window while recording")
    args = parser.parse_args()

    rrd_path, meta_path = make_run_paths(args.out_dir, args.label)
    record(rrd_path, meta_path, args.label, args.seconds,
           res=(args.width, args.height), fps=args.fps,
           enable_imu=not args.no_imu, spawn_viewer=not args.no_viewer)
