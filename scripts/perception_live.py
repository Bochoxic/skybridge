#!/usr/bin/env python3
"""Live drone detection and tracking from a RealSense, with a Rerun view.

Works on its own, with no radio or drone link. ``rc_fly.py --perception``
starts this same script as a child process.

Start it with the drone on the ground, or out of view: the first second learns
the static background. Take off only after the warm-up has finished.

Each estimate is also sent over UDP (see ``link`` in configs/perception.yaml).

Usage:
    python scripts/perception_live.py                     # camera + viewer
    python scripts/perception_live.py --viewer off        # camera, no viewer
    python scripts/perception_live.py --record --label test1
    python scripts/perception_live.py --replay data/realsense/<stem>.rrd
                                                          # a recording, in real time
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from skybridge import Bus  # noqa: E402
from skybridge.perception.config import PerceptionConfig  # noqa: E402
from skybridge.perception.link import EstimateSender  # noqa: E402
from skybridge.perception.pipeline import DepthPipeline  # noqa: E402
from skybridge.perception.runner import ESTIMATE_TOPIC, RESULT_TOPIC, run  # noqa: E402
from skybridge.perception.session import die_with_parent  # noqa: E402
from skybridge.perception.sources import RealSenseSource, RrdReplaySource  # noqa: E402

ROOT = Path(__file__).parent.parent
CONFIG = ROOT / "configs" / "perception.yaml"
RECORD_DIR = ROOT / "data" / "realsense"


def open_viewer(cfg, pipeline, args, run_id):
    """A Rerun stream showing this run, or None with the viewer off."""
    import rerun as rr

    from skybridge.perception.viz import APP_ID, LIVE_TIMELINE, RerunSink, make_blueprint

    color = cfg.camera.color
    blueprint = make_blueprint(cfg.viewer.trail_seconds, live=True, color=color,
                               detail=cfg.viewer.detail, depth=cfg.viewer.depth)
    rec =rr.RecordingStream(APP_ID, recording_id=run_id,
                             batcher_config=rr.ChunkBatcherConfig.LOW_LATENCY())
    if args.viewer == "spawn":
        rec.spawn(memory_limit=cfg.viewer.memory_limit, default_blueprint=blueprint)
    else:
        rec.connect_grpc()
        rec.send_blueprint(blueprint)
    sink = RerunSink(pipeline, recording=rec, timeline=LIVE_TIMELINE,
                     arrow_scale=cfg.viewer.arrow_scale, depth_stride=cfg.viewer.depth_stride,
                     depth_hz=cfg.viewer.depth_hz, color_hz=cfg.viewer.color_hz, log_mask=False,
                     detail=cfg.viewer.detail, log_depth=cfg.viewer.depth,
                     background_stride=cfg.viewer.background_stride)
    sink.log_static()
    return rec, sink


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--replay", help=".rrd to play in real time instead of the camera")
    ap.add_argument("--viewer", choices=("spawn", "connect", "off"), default="spawn")
    ap.add_argument("--run-id", help="Rerun recording id (default: from the clock)")
    ap.add_argument("--parent-pid", type=int, help="exit when this process dies (used by rc_fly)")
    ap.add_argument("--record", action="store_true",
                    help="also save full-resolution depth, replayable with drone_detect.py")
    ap.add_argument("--label", default="flight", help="name for the --record files")
    ap.add_argument("--color", action="store_true", help="add a low-rate RGB view")
    ap.add_argument("--cam-height", type=float, help="override pipeline.cam_height")
    ap.add_argument("--cam-pitch", type=float, help="override pipeline.cam_pitch")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    cfg = PerceptionConfig.from_yaml(args.config)
    params = cfg.pipeline
    if args.cam_height is not None:
        params = replace(params, cam_height=args.cam_height)
    if args.cam_pitch is not None:
        params = replace(params, cam_pitch=args.cam_pitch)
    if args.color:
        cfg = replace(cfg, camera=replace(cfg.camera, color=True))

    os.nice(5)  # the RC process wins any contention for the CPU

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    if args.parent_pid is not None:
        die_with_parent(args.parent_pid)

    run_id = args.run_id or datetime.now().strftime("live_%Y-%m-%d_%H-%M-%S")
    if args.replay:
        print(f"loading {args.replay} ...", flush=True)
        source = RrdReplaySource(args.replay, realtime=True)
    else:
        source = RealSenseSource(cfg.camera)
    pipeline = DepthPipeline(params, source.camera)

    # Subscribed first, so the estimate leaves before anything slower runs.
    bus = Bus()
    sender = EstimateSender(cfg.link.host, cfg.link.port)
    bus.subscribe(ESTIMATE_TOPIC, sender.handle)

    rec = viewer = None
    if args.viewer != "off":
        try:
            from skybridge.perception.viz import LatestWorker

            rec, sink = open_viewer(cfg, pipeline, args, run_id)
            viewer = LatestWorker(sink.log_result)
            viewer.start()
            bus.subscribe(RESULT_TOPIC, viewer.handle)
        except Exception as exc:  # estimates matter more than the picture
            logging.warning("no Rerun viewer: %s", exc)

    recorder = None
    if args.record:
        from skybridge.perception.viz import DepthRecorder

        RECORD_DIR.mkdir(parents=True, exist_ok=True)
        stem = f"{datetime.now():%Y-%m-%d_%H-%M-%S}_{args.label}"
        recorder = DepthRecorder(RECORD_DIR / f"{stem}.rrd",
                                 {"label": args.label, **source.meta},
                                 source.camera.depth_scale)
        bus.subscribe(RESULT_TOPIC, recorder.handle)
        print(f"recording to {recorder.path}", flush=True)

    last_print = [0.0]

    def report(result, stats) -> None:
        now = time.monotonic()
        if result.background_ready:
            print("background ready: clear for takeoff", flush=True)
        if now - last_print[0] >= 1.0:
            last_print[0] = now
            print(stats.summary(source.skipped), flush=True)

    try:
        stats = run(source, pipeline, bus, stop, on_result=report)
        print(f"done: {stats.summary(source.skipped)}", flush=True)
    finally:
        source.close()
        sender.close()
        if viewer is not None:
            print(f"viewer skipped {viewer.skipped} frames", flush=True)
            viewer.close()
        if recorder is not None:
            recorder.close(source.skipped)
            print(f"saved {recorder.path}", flush=True)
        if rec is not None:
            rec.flush()


if __name__ == "__main__":
    main()
