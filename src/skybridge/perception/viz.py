"""Everything that writes to Rerun.

Three independent pieces, none of which the pipeline knows about:

    RerunSink      detection and tracking (offline: every frame, live: decimated)
    RerunBusSink   RC and telemetry from the flight process, on its own thread
    DepthRecorder  full-resolution depth, in the format the offline tools read
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections import deque
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from ..bus import Bus
from ..units import tick_to_us
from .link import TOPIC as ESTIMATE_TOPIC
from .pipeline import DepthPipeline
from .sources import LatestSlot
from .types import DepthFrame, PerceptionResult

log = logging.getLogger(__name__)

APP_ID = "skybridge_live"
LIVE_TIMELINE = "monotonic"

VERDICT_COLORS = {"ok": [0, 220, 0], "few_points": [255, 220, 0],
                  "too_small": [0, 220, 220], "too_big": [150, 100, 255],
                  "outside_gate": [255, 60, 60]}


def make_blueprint(trail_seconds, *, live=False, color=False, detail=True, depth=True):
    """Viewer layout. Trails are drawn by Rerun at playback time: each frame logs
    one point / segment, and the view shows only the last trail_seconds of them."""
    timelines = (LIVE_TIMELINE, "wall_time") if live else ("sensor_time", "wall_time")
    window = rrb.VisibleTimeRanges([
        rrb.VisibleTimeRange(
            timeline, start=rrb.TimeRangeBoundary.cursor_relative(seconds=-trail_seconds),
            end=rrb.TimeRangeBoundary.cursor_relative())
        for timeline in timelines])
    overrides = {
        "world/trail/raw": [window, rr.Points3D.from_fields().visualizer()],
        "world/trail/filtered": [window, rr.LineStrips3D.from_fields().visualizer()],
        "world/trail/output": [window, rr.LineStrips3D.from_fields().visualizer()],
        # Whole flight: available, but hidden until switched on in the viewer
        "world/trajectory": [rrb.EntityBehavior(visible=False),
                             rr.LineStrips3D.from_fields().visualizer()],
    }
    plots = [rrb.TimeSeriesView(name="position (world)", origin="plots/position"),
             rrb.TimeSeriesView(name="velocity (world)", origin="plots/velocity")]
    if detail:
        plots.append(rrb.TimeSeriesView(name="association", origin="plots/association"))
    images = [rrb.Spatial2DView(name="depth", origin="camera/depth")] if depth else []
    if live:
        plots += [rrb.TimeSeriesView(name="rc (us)", origin="plots/rc"),
                  rrb.TimeSeriesView(name="control", origin="plots/control"),
                  rrb.TimeSeriesView(name="attitude", origin="plots/telemetry/attitude"),
                  rrb.TimeSeriesView(name="battery", origin="plots/telemetry/battery"),
                  rrb.TimeSeriesView(name="link", origin="plots/telemetry/link_stats")]
        if color:
            images.append(rrb.Spatial2DView(name="color", origin="camera/color"))
    else:
        images.append(rrb.Spatial2DView(name="foreground", origin="camera/foreground"))
    return rrb.Blueprint(rrb.Horizontal(
        rrb.Spatial3DView(name="world (z up)", origin="world", overrides=overrides),
        rrb.Vertical(
            *images,
            rrb.Tabs(*plots),
            rrb.TextLogView(name="association log", origin="log")),
        column_shares=[3, 2]))


class RerunSink:
    """Logs one :class:`PerceptionResult` at a time.

    With no `recording` it uses Rerun's global stream. The defaults log
    everything; a live run passes the decimation options.

    What loads a live viewer is the number of separate entities per frame, not
    the bytes: about 40 per tracked frame gave seconds of lag at only 2 MB/s.
    ``detail=False`` sends only the drone position, its trail and the position
    and velocity plots, on top of a background logged once.
    """

    def __init__(
        self,
        pipeline: DepthPipeline,
        *,
        recording: rr.RecordingStream | None = None,
        timeline: str = "sensor_time",
        arrow_scale: float = 0.3,
        depth_stride: int = 1,
        depth_hz: float = 0.0,      # 0 = every frame
        color_hz: float = 0.0,
        log_mask: bool = True,
        detail: bool = True,
        log_depth: bool = True,
        background_stride: int = 1,
    ) -> None:
        self.pipeline = pipeline
        self.rec = recording
        self.timeline = timeline
        self.arrow_scale = arrow_scale
        self.depth_stride = depth_stride
        self.depth_period = 1.0 / depth_hz if depth_hz > 0 else 0.0
        self.color_period = 1.0 / color_hz if color_hz > 0 else 0.0
        self.log_mask = log_mask
        self.detail = detail
        self.log_depth = log_depth
        self.background_stride = background_stride
        self._next_depth = float("-inf")
        self._next_color = float("-inf")
        self._last_filtered = None
        self._last_output = None
        self._background_logged = False
        self._cleared: set[str] = set()
        self._last_text = None

    # ── Plumbing ──────────────────────────────────────────────────────────────
    def _log(self, path, archetype, *, static=False):
        self._cleared.discard(path)
        rr.log(path, archetype, static=static, recording=self.rec)

    def _clear(self, path, *, recursive=False):
        """Without detail, an entity that is already clear isn't cleared again."""
        if self.detail or path not in self._cleared:
            self._log(path, rr.Clear(recursive=recursive))
            self._cleared.add(path)

    def _set_time(self, t, wall):
        rr.set_time(self.timeline, duration=t, recording=self.rec)
        rr.set_time("wall_time", timestamp=wall, recording=self.rec)

    def handle(self, topic: str, result: PerceptionResult, t: float) -> None:
        """Bus subscriber."""
        self.log_result(result)

    # ── What gets logged ──────────────────────────────────────────────────────
    def log_static(self):
        """The world frame, the camera pose and the crop box. Call once."""
        p = self.pipeline
        self._log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        # Camera-frame entities live under world/camera; Rerun applies the pose
        self._log("world/camera", rr.Transform3D(translation=p.cam_t, mat3x3=p.cam_R),
                  static=True)
        self._log("world/camera/axes", rr.Arrows3D(
            origins=[[0, 0, 0]] * 3, vectors=np.eye(3) * 0.2,
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            labels=["cam x (right)", "cam y (down)", "cam z (depth)"]), static=True)
        lo, hi = p.box
        self._log("world/camera/crop_box", rr.Boxes3D(centers=[(lo + hi) / 2],
                                                      sizes=[hi - lo]), static=True)
        if not self.detail:
            self._log("plots/position", rr.SeriesLines(names=["x", "y", "z"]), static=True)
            self._log("plots/velocity", rr.SeriesLines(names=["vx", "vy", "vz"]), static=True)

    def background_points(self) -> np.ndarray:
        """The learned background as 3D points, thinned by ``background_stride``."""
        p, s = self.pipeline, self.background_stride
        keep = np.zeros(p.background.z.shape, dtype=bool)
        keep[::s, ::s] = True
        keep &= ~np.isnan(p.background.z)
        return p.project.points(p.background.z.ravel(), keep.ravel())

    def _log_images(self, frame: DepthFrame):
        if self.log_depth and frame.t >= self._next_depth:
            self._next_depth = frame.t + self.depth_period
            s = self.depth_stride
            self._log("camera/depth", rr.DepthImage(
                frame.depth if s == 1 else frame.depth[::s, ::s],
                meter=1.0 / self.pipeline.camera.depth_scale))
        if frame.color is not None and frame.t >= self._next_color:
            self._next_color = frame.t + self.color_period
            rgb = np.ascontiguousarray(frame.color[:, :, ::-1])
            self._log("camera/color", rr.Image(rgb).compress(jpeg_quality=80))

    def log_prediction(self, t, wall, out):
        """A smoothed position between depth frames (offline ``--predict-hz``)."""
        self._set_time(t, wall)
        self._log("world/output", rr.Points3D([out], radii=0.02, colors=[255, 255, 255]))
        if self._last_output is not None:
            self._log("world/trail/output", rr.LineStrips3D(
                [[self._last_output, out]], radii=0.003, colors=[255, 255, 255]))
        for axis, v in zip("xyz", out):
            self._log(f"plots/position/{axis}_output", rr.Scalars(v))
        self._last_output = out

    def log_result(self, res: PerceptionResult):
        # Checked on the background, not the result: live, the frame that
        # finished the warm-up may be one the viewer thread skipped.
        p = self.pipeline
        if not self._background_logged and p.background is not None and p.background.z is not None:
            self._background_logged = True
            self._log("world/camera/background", rr.Points3D(
                self.background_points(), radii=0.004, colors=[90, 90, 90]), static=True)

        self._set_time(res.frame.t, res.frame.wall)
        self._log_images(res.frame)
        if res.warmup:
            return

        if self.log_mask and res.foreground is not None:
            self._log("camera/foreground", rr.Image(
                res.foreground.reshape(self.pipeline.background.z.shape).astype(np.uint8) * 255))

        if self.detail:
            self._log("world/camera/cropped", rr.Points3D(
                res.cropped, radii=0.005, colors=[150, 150, 150]))
            if res.cluster is not None:
                color = [0, 220, 0] if res.detected else [255, 160, 0]
                self._log("world/camera/cluster", rr.Points3D(
                    res.cluster, radii=0.006, colors=color))
            else:
                self._log("world/camera/cluster", rr.Clear(recursive=False))
            self._log_clusters(res.diag["clusters"])
            self._log_association(res)
            self._log_output(res)
        else:
            self._log_transition(res)
        self._log_track(res)

    def _log_transition(self, res: PerceptionResult):
        """One line each time the status or reason changes, not one per frame."""
        status, reason = res.estimate.status, res.diag["reason"]
        if (status, reason) != self._last_text:
            self._last_text = (status, reason)
            level = "WARN" if status in ("coasting", "lost") else "INFO"
            self._log("log/association", rr.TextLog(f"{status}: {reason}", level=level))

    def _log_clusters(self, clusters):
        # Every DBSCAN cluster, coloured by verdict, labelled with the numbers
        if not clusters:
            self._log("world/camera/clusters", rr.Clear(recursive=True))
            return
        self._log("world/camera/clusters", rr.Points3D(
            np.concatenate([c["points"] for c in clusters]), radii=0.004,
            colors=np.concatenate([[VERDICT_COLORS[c["verdict"]]] * c["n"] for c in clusters])))
        self._log("world/camera/clusters/labels", rr.Points3D(
            [c["centroid"] for c in clusters], radii=0.01,
            colors=[VERDICT_COLORS[c["verdict"]] for c in clusters],
            labels=[f"{c['verdict']} n={c['n']} ext={c['extent']:.2f}"
                    + ("" if c["dist"] is None else f" d={c['dist']:.2f}")
                    for c in clusters], show_labels=True))

    def _log_association(self, res: PerceptionResult):
        diag, status = res.diag, res.estimate.status
        for k in ("cand_dist", "near_dist", "since_update"):
            if diag.get(k) is not None:
                self._log(f"plots/association/{k}", rr.Scalars(diag[k]))
        if diag.get("since_update") is not None:
            self._log("plots/association/gate", rr.Scalars(diag["gate"]))
            self._log("plots/association/max_coast", rr.Scalars(self.pipeline.params.max_coast))
        if diag.get("near_pts") is not None:
            self._log("plots/association/near_pts", rr.Scalars(diag["near_pts"]))
        if diag["reason"] != "ok" or status != "detected":
            parts = [f"frame {res.frame.seq} {status}: {diag['reason']}"]
            for k, fmt in (("cand_dist", "d={:.3f}"), ("gate", "gate={:.2f}"),
                           ("cand_n", "n={}"), ("cand_extent", "ext={:.3f}"),
                           ("near_pts", "pts_in_gate={}"),
                           ("near_dist", "closest_pt={:.3f}"),
                           ("since_update", "since_update={:.2f}s")):
                if diag.get(k) is not None:
                    parts.append(fmt.format(diag[k]))
            level = "WARN" if status in ("coasting", "lost") else "INFO"
            self._log("log/association", rr.TextLog(" ".join(parts), level=level))

    def _log_output(self, res: PerceptionResult):
        out = res.output
        if out is None:
            self._log("world/output", rr.Clear(recursive=False))
            self._last_output = None
            return
        self._log("world/output", rr.Points3D([out], radii=0.02, colors=[255, 255, 255]))
        if self._last_output is not None:
            self._log("world/trail/output", rr.LineStrips3D(
                [[self._last_output, out]], radii=0.003, colors=[255, 255, 255]))
        for axis, v in zip("xyz", out):
            self._log(f"plots/position/{axis}_output", rr.Scalars(v))
        self._last_output = out

    def _log_track(self, res: PerceptionResult):
        if self.detail:
            if res.detected:
                self._log("world/drone", rr.Points3D([res.world_pos], radii=0.03,
                                                     colors=[255, 0, 0]))
                for axis, v in zip("xyz", res.world_pos):
                    self._log(f"plots/position/{axis}_raw", rr.Scalars(v))
                # Raw detections: one dot per frame, windowed by the blueprint
                self._log("world/trail/raw", rr.Points3D(
                    [res.world_pos], radii=0.008, colors=[255, 0, 0]))
            else:
                self._log("world/drone", rr.Clear(recursive=False))

            if res.prediction is not None:
                # Where the tracker searched this frame: prediction + gate
                self._log("world/prediction", rr.Ellipsoids3D(
                    centers=[res.prediction], half_sizes=[[res.diag["gate"]] * 3],
                    colors=[255, 0, 255], line_radii=0.001,
                    fill_mode=rr.components.FillMode.MajorWireframe))
            else:
                self._log("world/prediction", rr.Clear(recursive=False))

        pos, vel = res.filtered, res.velocity
        if pos is None:
            self._last_filtered = None
            self._clear("world/filtered", recursive=True)
            return
        if self._last_filtered is not None:
            # Blue: updated by a detection. Magenta: prediction only
            color = [0, 120, 255] if res.detected else [255, 0, 255]
            self._log("world/trail/filtered", rr.LineStrips3D(
                [[self._last_filtered, pos]], radii=0.004, colors=[color]))
        self._last_filtered = pos.copy()
        self._log("world/filtered", rr.Points3D([pos], radii=0.025, colors=[0, 120, 255]))
        if not self.detail:
            self._log("plots/position", rr.Scalars(pos))
            self._log("plots/velocity", rr.Scalars(vel))
            return
        self._log("world/filtered/velocity", rr.Arrows3D(
            origins=[pos], vectors=[vel * self.arrow_scale], colors=[0, 120, 255]))
        for axis, p_, v in zip("xyz", pos, vel):
            self._log(f"plots/position/{axis}", rr.Scalars(p_))
            self._log(f"plots/velocity/v{axis}", rr.Scalars(v))
        self._log("plots/velocity/speed", rr.Scalars(np.linalg.norm(vel)))


class LatestWorker:
    """Calls `handler` on its own thread, with only the newest item.

    Viewer logging can take as long as detection and can block outright if the
    viewer falls behind. Behind this, it costs the detection loop nothing: the
    viewer just shows fewer frames.
    """

    def __init__(self, handler: Callable[[Any], None]) -> None:
        self._handler = handler
        self._slot: LatestSlot[Any] = LatestSlot()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="skybridge-viewer", daemon=True)

    @property
    def skipped(self) -> int:
        return self._slot.overwritten

    def start(self) -> None:
        self._thread.start()

    def handle(self, topic: str, item: Any, t: float) -> None:
        """Bus subscriber. Never blocks."""
        self._slot.put(item)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            item = self._slot.get(0.1)
            if item is None:
                continue
            try:
                self._handler(item)
            except Exception:
                log.exception("viewer logging failed")


class RerunBusSink:
    """Logs RC and telemetry from the Bus to Rerun without slowing the RC thread.

    Bus callbacks run on the RC thread, so the callback only appends to a
    bounded queue. A separate thread does the logging. If that thread falls
    behind, the oldest messages are dropped.
    """

    def __init__(
        self,
        bus: Bus,
        recording: rr.RecordingStream | None,
        channel_names: Mapping[int, str],
        *,
        maxlen: int = 4096,
        log_fn: Callable[[str, Any, float], None] | None = None,
        log_estimates: bool = False,
    ) -> None:
        self._topics = ("rc/sent", "telemetry/", "control/status") + (
            (ESTIMATE_TOPIC,) if log_estimates else ())
        self._last_control: dict[str, tuple[bool, str]] = {}
        self._last_status: tuple[str, str] | None = None
        self._last_pos: list[float] | None = None
        self._bus = bus
        self._rec = recording
        self._names = dict(channel_names)
        self._queue: deque[tuple[str, Any, float]] = deque(maxlen=maxlen)
        self._log_fn = log_fn or self._log_message
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._unsubscribe: list[Callable[[], None]] = []
        self._named: set[str] = set()
        self.dropped = 0

    def start(self) -> None:
        self._unsubscribe = [self._bus.subscribe(prefix, self._on_message)
                             for prefix in self._topics]
        self._thread = threading.Thread(target=self._run, name="skybridge-rerun-bus", daemon=True)
        self._thread.start()

    def close(self) -> None:
        for unsubscribe in self._unsubscribe:
            unsubscribe()
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _on_message(self, topic: str, payload: Any, t: float) -> None:
        if len(self._queue) == self._queue.maxlen:
            self.dropped += 1
        self._queue.append((topic, payload, t))

    def _run(self) -> None:
        while not self._stop.is_set():
            self._drain()
            self._stop.wait(0.02)
        self._drain()

    def _drain(self) -> None:
        while self._queue:
            topic, payload, t = self._queue.popleft()
            try:
                self._log_fn(topic, payload, t)
            except Exception:
                log.exception("failed to log %r to Rerun", topic)

    def _log_message(self, topic: str, payload: Any, t: float) -> None:
        rr.set_time(LIVE_TIMELINE, duration=t, recording=self._rec)
        # Also on wall time, the one clock shared with a saved depth recording.
        rr.set_time("wall_time", timestamp=time.time() - (time.monotonic() - t),
                    recording=self._rec)
        if topic == ESTIMATE_TOPIC:
            self._log_estimate(payload)
            return
        if topic == "rc/sent":
            self._log_series("plots/rc", list(self._names.values()),
                             [tick_to_us(payload[channel]) for channel in self._names])
            return
        if topic == "control/status":
            self._log_control(payload)
            return
        if "mode" in payload:
            rr.log("log/flight_mode", rr.TextLog(payload["mode"]), recording=self._rec)
        numeric = {k: v for k, v in payload.items() if not isinstance(v, (str, bool))}
        self._log_series(f"plots/telemetry/{topic.removeprefix('telemetry/')}",
                         list(numeric), list(numeric.values()))

    def _log_control(self, status: Any) -> None:
        """A controller's internals for tuning, and a log line when its state changes."""
        state = (status.engaged, status.note)
        if state != self._last_control.get(status.name):
            self._last_control[status.name] = state
            text = "engaged" if status.engaged else (status.note or "off")
            level = "WARN" if status.note else "INFO"
            rr.log("log/control", rr.TextLog(f"{status.name}: {text}", level=level),
                   recording=self._rec)
        self._log_series(f"plots/control_engaged/{status.name}", ["engaged"],
                         [float(status.engaged)])
        if status.debug:  # one entity: cheaper, and setpoint and z share a plot
            self._log_series(f"plots/control/{status.name}", list(status.debug),
                             list(status.debug.values()))

    def _log_estimate(self, e: Any) -> None:
        """The drone's state, on the same paths as the live view."""
        if (e.status, e.reason) != self._last_status:
            self._last_status = (e.status, e.reason)
            level = "WARN" if e.status in ("coasting", "lost") else "INFO"
            rr.log("log/association", rr.TextLog(f"{e.status}: {e.reason}", level=level),
                   recording=self._rec)
        if not e.valid:
            if self._last_pos is not None:
                rr.log("world/filtered", rr.Clear(recursive=True), recording=self._rec)
            self._last_pos = None
            return

        pos = list(e.pos)
        rr.log("world/filtered", rr.Points3D([pos], radii=0.025, colors=[0, 120, 255]),
               recording=self._rec)
        if self._last_pos is not None:
            color = [0, 120, 255] if e.status == "detected" else [255, 0, 255]
            rr.log("world/trail/filtered", rr.LineStrips3D(
                [[self._last_pos, pos]], radii=0.004, colors=[color]), recording=self._rec)
        self._last_pos = pos
        self._log_series("plots/position", ["x", "y", "z"], e.pos)
        self._log_series("plots/velocity", ["vx", "vy", "vz"], e.vel)
        self._log_series("plots/position_std", ["x", "y", "z"], e.pos_std)
        self._log_series("plots/since_update", ["since_update"], [e.since_update])

    def _log_series(self, path: str, names: Sequence[str], values: Sequence[float]) -> None:
        """All of one message's values as a single entity, which is far cheaper to ingest."""
        if not values:
            return
        if path not in self._named:
            self._named.add(path)
            rr.log(path, rr.SeriesLines(names=names), static=True, recording=self._rec)
        rr.log(path, rr.Scalars(values), recording=self._rec)


class DepthRecorder:
    """Writes full-resolution depth so a live run can be replayed with drone_detect.py.

    Produces the same ``.rrd`` entity and timelines and the same JSON sidecar
    as ``realsense_record.py``. Only frames the pipeline processed are kept.
    """

    def __init__(self, rrd_path: str | Path, meta: Mapping[str, Any], depth_scale: float) -> None:
        self.path = Path(rrd_path)
        self.meta_path = self.path.with_suffix(".json")
        self.meta = {"start_wall_time": time.time(), **meta}
        self._meter = 1.0 / depth_scale
        self._rec = rr.RecordingStream("realsense_d455", recording_id=self.path.stem)
        self._rec.save(self.path)
        self._t0: float | None = None
        self.n_frames = 0
        self._write_meta()

    def _write_meta(self) -> None:
        self.meta_path.write_text(json.dumps(self.meta, indent=2))

    def handle(self, topic: str, result: PerceptionResult, t: float) -> None:
        frame = result.frame
        if self._t0 is None:
            self._t0 = frame.t
        rr.set_time("sensor_time", duration=frame.t - self._t0, recording=self._rec)
        rr.set_time("wall_time", timestamp=frame.wall, recording=self._rec)
        rr.log("camera/depth", rr.DepthImage(frame.depth, meter=self._meter), recording=self._rec)
        self.n_frames += 1

    def close(self, skipped: int = 0) -> None:
        self._rec.flush()
        self._rec.disconnect()
        self.meta["frames_logged"] = {"depth": self.n_frames}
        self.meta["frames_dropped"] = {"depth": skipped}
        self._write_meta()
