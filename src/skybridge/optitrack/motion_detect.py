# -*- coding: utf-8 -*-
"""
Offline 2D motion detection on the RGB stream of a RealSense recording made
with realsense_record.py. The camera and background are static, so pixels
that change are the drone (or its shadow / reflections). Image only: this is
not merged with the 3D depth detection yet.

Pipeline per color frame:
    1. MOG2 background subtraction -> foreground mask
    2. morphological open (remove speckle) + close (fill the blob)
    3. connected components, drop blobs smaller than --min-area pixels
    4. the largest blob is the candidate drone: centroid (u, v) and bbox

Run:
    uv run motion_detect.py ../../../data/realsense/2026-10-28_20-40-52_moving_drone.rrd

Outputs, next to the recording:
    <stem>_motion.csv  -- one row per color frame
    <stem>_motion.rrd  -- color image with blob boxes, foreground mask,
                          foreground pixel count over time

Both this and drone_detect.py log under the same Rerun recording id, so
opening the two outputs together shows them on one timeline:
    rerun <stem>_detections.rrd <stem>_motion.rrd
"""
import argparse
import csv
import time
from pathlib import Path

import cv2
import numpy as np
import rerun as rr
import rerun.chunk as rrc

CSV_FIELDS = ["frame", "wall_time", "sensor_time", "fg_pixels", "n_blobs",
              "u", "v", "box_x", "box_y", "box_w", "box_h", "area"]


def iter_color_frames(rrd_path):
    """Yield (wall_time_s, sensor_time_s, jpeg bytes) for every color frame."""
    for chunk in rrc.RrdReader(str(rrd_path)).stream():
        if chunk.entity_path != "/camera/color" or chunk.is_static:
            continue
        rb = chunk.to_record_batch()
        wall = rb.column("wall_time").cast("int64").to_numpy() / 1e9
        sensor = rb.column("sensor_time").cast("int64").to_numpy() / 1e9
        blobs = rb.column("EncodedImage:blob").to_pylist()
        for i in range(rb.num_rows):
            if blobs[i]:
                yield wall[i], sensor[i], bytes(blobs[i][0])


def detect(rrd_path, args):
    rrd_path = Path(rrd_path)
    csv_path = rrd_path.with_name(rrd_path.stem + "_motion.csv")
    out_rrd = rrd_path.with_name(rrd_path.stem + "_motion.rrd")

    log = args.viewer or not args.no_rrd
    if log:
        rr.init("drone_detect", recording_id=rrd_path.stem, spawn=args.viewer)
        if not args.no_rrd:
            rr.save(out_rrd)

    subtractor = cv2.createBackgroundSubtractorMOG2(
        history=args.history, varThreshold=args.var_threshold, detectShadows=False)
    k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (args.open_px,) * 2)
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (args.close_px,) * 2)

    n_frames = n_detected = 0
    t_start = time.time()
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for frame, (wall, sensor, jpeg) in enumerate(iter_color_frames(rrd_path)):
            if args.max_frames and frame >= args.max_frames:
                break
            n_frames += 1
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)

            mask = subtractor.apply(img, learningRate=args.learning_rate)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)

            n, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
            # label 0 is the background
            keep = [i for i in range(1, n)
                    if stats[i, cv2.CC_STAT_AREA] >= args.min_area]
            keep.sort(key=lambda i: stats[i, cv2.CC_STAT_AREA], reverse=True)

            row = {"frame": frame, "wall_time": f"{wall:.6f}",
                   "sensor_time": f"{sensor:.6f}",
                   "fg_pixels": int(np.count_nonzero(mask)), "n_blobs": len(keep)}
            # Too early: the background model hasn't settled yet
            warm = frame >= args.warmup
            if keep and warm:
                best = keep[0]
                x, y, w, h, area = stats[best]
                u, v = centroids[best]
                row.update({"u": f"{u:.1f}", "v": f"{v:.1f}", "box_x": x, "box_y": y,
                            "box_w": w, "box_h": h, "area": area})
                n_detected += 1
            writer.writerow(row)

            if log:
                rr.set_time("sensor_time", duration=sensor)
                rr.set_time("wall_time", timestamp=wall)
                rr.log("camera/color", rr.EncodedImage(contents=jpeg,
                                                       media_type="image/jpeg"))
                rr.log("camera/motion_mask", rr.Image(mask))
                rr.log("plots/motion/fg_pixels", rr.Scalars(row["fg_pixels"]))
                if keep and warm:
                    boxes = stats[keep, :4]
                    colors = [[255, 0, 0]] + [[255, 200, 0]] * (len(keep) - 1)
                    rr.log("camera/color/blobs", rr.Boxes2D(
                        mins=boxes[:, :2], sizes=boxes[:, 2:], colors=colors))
                    rr.log("camera/color/drone_2d", rr.Points2D(
                        [[u, v]], radii=4, colors=[255, 0, 0]))
                else:
                    rr.log("camera/color/blobs", rr.Clear(recursive=False))
                    rr.log("camera/color/drone_2d", rr.Clear(recursive=False))

    elapsed = time.time() - t_start
    print(f"Frames: {n_frames}, motion blob in {n_detected} "
          f"({100 * n_detected / max(n_frames, 1):.1f}%), "
          f"{1000 * elapsed / max(n_frames, 1):.1f} ms/frame")
    print(f"Wrote {csv_path}")
    if not args.no_rrd:
        print(f"Wrote {out_rrd} -- view with: rerun {out_rrd}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", help=".rrd from realsense_record.py")
    bg = parser.add_argument_group("background subtraction (OpenCV MOG2)")
    bg.add_argument("--history", type=int, default=500,
                    help="frames the background model remembers")
    bg.add_argument("--var-threshold", type=float, default=25.0,
                    help="pixel-vs-model distance to count as foreground; "
                         "higher = less sensitive")
    bg.add_argument("--learning-rate", type=float, default=-1,
                    help="background update rate, -1 = automatic (1/history); "
                         "lower keeps a hovering drone from fading into the background")
    bg.add_argument("--warmup", type=int, default=30,
                    help="frames to learn the background before reporting blobs")
    blob = parser.add_argument_group("blobs")
    blob.add_argument("--open-px", type=int, default=3,
                      help="opening kernel size (removes speckle)")
    blob.add_argument("--close-px", type=int, default=9,
                      help="closing kernel size (fills holes in the blob)")
    blob.add_argument("--min-area", type=int, default=30,
                      help="smallest blob kept, pixels")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="stop after this many frames (0 = all)")
    parser.add_argument("--viewer", action="store_true",
                        help="spawn a live Rerun viewer while processing")
    parser.add_argument("--no-rrd", action="store_true",
                        help="don't write the _motion.rrd visualization")
    args = parser.parse_args()
    detect(args.recording, args)
