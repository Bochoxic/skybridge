"""The live perception path: pipeline, output contract, link and sinks."""
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from skybridge import Bus
from skybridge.config import ConfigError, DroneConfig
from skybridge.perception import CameraModel, DepthFrame, DroneEstimate, PerceptionResult
from skybridge.perception import detector
from skybridge.perception import session as session_module
from skybridge.perception.session import PerceptionSession
from skybridge.perception.config import PerceptionConfig, PipelineParams
from skybridge.perception.geometry import camera_to_world
from skybridge.perception.link import EstimateSender, PerceptionLink, vis_status
from skybridge.perception.pipeline import DepthPipeline
from skybridge.perception.runner import ESTIMATE_TOPIC, RESULT_TOPIC, run
from skybridge.perception.sources import (
    FakeFrameSource,
    LatestSlot,
    RrdReplaySource,
    capture_monotonic,
)
from skybridge.perception.tracker import Tracker
from skybridge.perception.viz import DepthRecorder, LatestWorker, RerunBusSink, RerunSink
from skybridge.state import Timestamped

FPS = 30.0
CAMERA = CameraModel({"width": 64, "height": 48, "fx": 50.0, "fy": 50.0,
                      "ppx": 32.0, "ppy": 24.0}, 0.001)
PARAMS = PipelineParams(bg_frames=5, stride=1, min_points=8, persist=2)


def scene(column=None, depth_mm=1500):
    """A wall at 3 m (outside the crop box), optionally with a 10x10 px drone."""
    depth = np.full((48, 64), 3000, dtype=np.uint16)
    if column is not None:
        depth[19:29, column:column + 10] = depth_mm
    return depth


def frames(n_empty=5, n_drone=10, start=0.0):
    out = [DepthFrame(scene(), start + i / FPS, seq=i) for i in range(n_empty)]
    out += [DepthFrame(scene(10 + i), start + (n_empty + i) / FPS, seq=n_empty + i)
            for i in range(n_drone)]
    return out


def estimate(**kw):
    defaults = dict(t=10.0, seq=1, status="detected", valid=True, pos=(1.0, 2.0, 3.0),
                    vel=(1.0, 0.0, -1.0), pos_std=(0.01, 0.01, 0.01), since_update=0.0,
                    reason="ok")
    return DroneEstimate(**{**defaults, **kw})


# ── Pipeline ──────────────────────────────────────────────────────────────────
def test_pipeline_warms_up_then_tracks():
    pipeline = DepthPipeline(PARAMS, CAMERA)
    results = [pipeline.process(f) for f in frames()]
    assert [r.estimate.status for r in results[:5]] == ["warmup"] * 5
    assert [r.background_ready for r in results[:5]] == [False] * 4 + [True]
    assert results[5].estimate.status == "candidate" and not results[5].estimate.valid
    assert results[6].estimate.status == "detected" and results[6].estimate.valid
    assert all(r.estimate.status == "detected" for r in results[6:])


def test_estimate_describes_the_track():
    pipeline = DepthPipeline(PARAMS, CAMERA)
    last = [pipeline.process(f) for f in frames()][-1]
    e = last.estimate
    assert e.t == last.frame.t and e.seq == last.frame.seq and e.reason == "ok"
    assert e.pos == pytest.approx(tuple(last.output))
    assert e.vel == pytest.approx(tuple(last.velocity))
    assert all(s > 0 for s in e.pos_std) and e.since_update == 0.0


def test_estimate_while_coasting_reports_time_since_update():
    tracker = Tracker(PARAMS, *camera_to_world(0.0, 0.0))
    g = np.arange(5) * 0.01
    gx, gy, gz = np.meshgrid(g, g, g[:2], indexing="ij")
    blob = np.column_stack((gx.ravel(), gy.ravel(), gz.ravel() + 1.5))
    for i in range(4):
        tracker.step(i / FPS, blob)
    t = 3 / FPS + 0.1
    status = tracker.step(t, np.empty((0, 3)))[1]
    e = tracker.estimate(t, 9, status)
    assert e.status == "coasting" and e.valid
    assert e.since_update == pytest.approx(0.1)


def test_estimate_without_track_is_invalid():
    tracker = Tracker(PARAMS, *camera_to_world(0.0, 0.0))
    status = tracker.step(0.0, np.empty((0, 3)))[1]
    e = tracker.estimate(0.0, 0, status)
    assert not e.valid and e.pos is None and e.at(1.0) is None


# ── Output contract ───────────────────────────────────────────────────────────
def test_at_projects_at_constant_velocity():
    assert estimate().at(10.1) == pytest.approx((1.1, 2.0, 2.9))


def test_at_is_limited_and_never_goes_backwards():
    e = estimate()
    assert e.at(99.0, max_ahead=0.2) == pytest.approx((1.2, 2.0, 2.8))
    assert e.at(9.0) == pytest.approx(e.pos)


def test_wire_round_trip():
    e = estimate()
    assert DroneEstimate.from_wire(e.to_wire()) == e
    invalid = estimate(valid=False, pos=None, vel=None, pos_std=None, since_update=None)
    assert DroneEstimate.from_wire(invalid.to_wire()) == invalid


@pytest.mark.parametrize("data", [b"", b"not json", b"[1, 2]", b'{"v": 99}', b'{"v": 1, "t": 1}'])
def test_malformed_wire_is_rejected(data):
    with pytest.raises(ValueError):
        DroneEstimate.from_wire(data)


# ── Point cap ─────────────────────────────────────────────────────────────────
CAP_ARGS = NS(min_points=1, eps=0.05, min_samples=5, no_bg=False, min_size=0.0, max_size=0.4,
              ambiguity_ratio=0.5, gate=0.3, max_points=100)


def clutter(n=500, center=(0.0, 0.0, 1.5)):
    return np.random.default_rng(0).normal(size=(n, 3)) * 0.01 + np.array(center)


def test_too_many_points_without_a_reference_skips_clustering(monkeypatch):
    monkeypatch.setattr(detector, "DBSCAN", lambda **_: pytest.fail("DBSCAN must not run"))
    cluster, status, _, diag = detector.select_candidate(clutter(), None, CAP_ARGS)
    assert cluster is None and status == "none" and diag["reason"] == "too_many_points"


def test_over_the_cap_clusters_only_near_the_reference():
    drone = clutter(50)
    person = clutter(500, center=(0.8, 0.0, 1.5))
    points = np.vstack((drone, person))
    cluster, status, *_ = detector.select_candidate(points, np.array([0.0, 0.0, 1.5]), CAP_ARGS)
    assert status == "candidate" and len(cluster) == 50


def test_too_many_points_even_near_the_reference():
    _, _, _, diag = detector.select_candidate(clutter(), np.array([0.0, 0.0, 1.5]), CAP_ARGS)
    assert diag["reason"] == "too_many_points"


def test_cap_of_zero_means_no_limit():
    args = NS(**{**vars(CAP_ARGS), "max_points": 0})
    cluster, *_ = detector.select_candidate(clutter(), None, args)
    assert cluster is not None and len(cluster) == 500


# ── Runner ────────────────────────────────────────────────────────────────────
def test_runner_publishes_the_estimate_before_the_result():
    bus, seen = Bus(), []
    bus.subscribe("perception/", lambda topic, payload, t: seen.append(topic))
    stats = run(FakeFrameSource(frames(), CAMERA), DepthPipeline(PARAMS, CAMERA), bus,
                threading.Event())
    assert stats.processed == 15
    assert seen[:2] == [ESTIMATE_TOPIC, RESULT_TOPIC] and len(seen) == 30


def test_a_failing_viewer_does_not_stop_estimates():
    bus, estimates = Bus(), []

    def broken(topic, payload, t):
        raise RuntimeError("viewer went away")

    bus.subscribe(ESTIMATE_TOPIC, lambda topic, e, t: estimates.append(e))
    bus.subscribe(RESULT_TOPIC, broken)
    run(FakeFrameSource(frames(), CAMERA), DepthPipeline(PARAMS, CAMERA), bus, threading.Event())
    assert len(estimates) == 15


def test_runner_stops_when_asked():
    stop = threading.Event()
    stop.set()
    stats = run(FakeFrameSource(frames(), CAMERA), DepthPipeline(PARAMS, CAMERA), Bus(), stop)
    assert stats.processed == 0


# ── Live frame handling ───────────────────────────────────────────────────────
def test_latest_slot_keeps_only_the_newest():
    slot = LatestSlot()
    slot.put(1)
    slot.put(2)
    slot.put(3)
    assert slot.get(0.0) == 3 and slot.overwritten == 2
    assert slot.get(0.0) is None


def test_capture_time_converts_epoch_to_monotonic():
    assert capture_monotonic(1_000_000_000.0, 50.0, 1_000_000.05) == pytest.approx(49.95)


@pytest.mark.parametrize("timestamp_ms", [12345.0, 1_000_000_000.0 - 5000, 1_000_000_000.0 + 5000])
def test_implausible_capture_time_is_refused(timestamp_ms):
    assert capture_monotonic(timestamp_ms, 50.0, 1_000_000.0) is None


# ── Link ──────────────────────────────────────────────────────────────────────
class Clock:
    def __init__(self, now):
        self.now = now

    def __call__(self):
        return self.now


def make_link(bus=None, clock=None):
    return PerceptionLink("127.0.0.1", 0, bus, **({"clock": clock} if clock else {}))


def wait_until(condition, timeout=2.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.005)
    return condition()


def test_udp_round_trip_reaches_latest_and_the_bus():
    bus, seen = Bus(), []
    bus.subscribe(ESTIMATE_TOPIC, lambda topic, e, t: seen.append((e, t)))
    link = make_link(bus)
    link.start()
    sender = EstimateSender("127.0.0.1", link.port)
    try:
        sender.send(estimate())
        assert wait_until(lambda: link.latest() is not None)
        assert link.latest().value == estimate()
        assert seen == [(estimate(), 10.0)]
    finally:
        sender.close()
        link.close()


def test_lost_datagrams_are_harmless_and_late_ones_ignored():
    link = make_link()
    try:
        link.receive(estimate(t=1.0, seq=1).to_wire())
        link.receive(estimate(t=3.0, seq=3).to_wire())      # 2 was lost
        link.receive(estimate(t=2.0, seq=2).to_wire())      # arrives late
        assert link.latest().value.seq == 3
        assert link.received == 2 and link.rejected == 1
    finally:
        link.close()


def test_garbage_datagram_is_counted_not_fatal():
    link = make_link()
    try:
        link.receive(b"garbage")
        assert link.rejected == 1 and link.latest() is None
    finally:
        link.close()


def test_sending_with_no_receiver_does_not_raise():
    sender = EstimateSender("127.0.0.1", 9)
    sender.send(estimate())
    sender.close()


def test_a_stalled_camera_shows_up_as_age():
    clock = Clock(10.05)
    link = make_link(clock=clock)
    try:
        link.receive(estimate().to_wire())
        assert link.latest().age == pytest.approx(0.05)
        assert not link.latest().is_stale(0.5)
        clock.now = 12.0
        assert link.latest().is_stale(0.5)
    finally:
        link.close()


def stamped(e, age):
    return Timestamped(e, e.t).at(e.t + age)


def test_status_text():
    assert vis_status(None) == "VIS waiting"
    assert vis_status(stamped(estimate(), 2.0)) == "VIS LOST 2.0s"
    assert vis_status(stamped(estimate(status="warmup", valid=False, pos=None), 0.0)) == "VIS warmup"
    assert vis_status(stamped(estimate(status="none", valid=False, pos=None), 0.0)) == "VIS none"
    assert vis_status(stamped(estimate(), 0.045)) == "VIS z=3.00m v=-1.0 detected 45ms"


def test_background_learning_pause_is_not_reported_as_lost():
    warmup = estimate(status="warmup", valid=False, pos=None)
    assert vis_status(stamped(warmup, 0.6)) == "VIS warmup"
    assert vis_status(stamped(warmup, 3.0)) == "VIS LOST 3.0s"


# ── Sinks ─────────────────────────────────────────────────────────────────────
def test_bus_sink_never_blocks_the_publisher():
    release = threading.Event()
    logged = []

    def slow_log(topic, payload, t):
        release.wait(5.0)
        logged.append(t)

    bus = Bus()
    sink = RerunBusSink(bus, None, {0: "roll"}, maxlen=10, log_fn=slow_log)
    sink.start()
    try:
        started = time.perf_counter()
        for i in range(1000):
            bus.publish("rc/sent", (0,) * 16, float(i))
        assert time.perf_counter() - started < 0.5
        assert sink.dropped > 900
    finally:
        release.set()
        sink.close()
    assert logged[-1] == 999.0  # drops the oldest, keeps the newest


def test_bus_sink_ignores_other_topics():
    logged = []
    bus = Bus()
    sink = RerunBusSink(bus, None, {}, log_fn=lambda topic, p, t: logged.append(topic))
    sink.start()
    try:
        bus.publish("rc/input", object(), 0.0)
        bus.publish("telemetry/attitude", {"roll_deg": 1.0}, 0.0)
    finally:
        sink.close()
    assert logged == ["telemetry/attitude"]


def test_recorded_session_saves_rc_and_telemetry_to_a_file(tmp_path, monkeypatch):
    import rerun.chunk as rrc

    monkeypatch.setattr(session_module, "LOG_DIR", tmp_path)
    bus = Bus()
    drone = DroneConfig.from_dict({"modes": {"arm": {"aux": 1, "range": [1700, 2100]}}})
    session = PerceptionSession(bus, drone, viewer=False, record=True)
    assert session._start_logging() == "off"  # no viewer for the child to reach
    bus.publish("rc/sent", (992,) * 16, 1.0)
    bus.publish("telemetry/attitude", {"roll_deg": 1.5}, 1.0)
    link = make_link(bus)  # states reach the Bus through the link, as in a real run
    link.receive(estimate(t=10.0, seq=1).to_wire())
    link.receive(estimate(t=10.1, seq=2).to_wire())
    link.close()
    session.close()

    path = tmp_path / f"{session.run_id}_flight.rrd"
    entities = {c.entity_path for c in rrc.RrdReader(str(path)).stream() if not c.is_static}
    assert {"/plots/rc", "/plots/telemetry/attitude", "/plots/position", "/plots/velocity",
            "/plots/position_std", "/world/filtered", "/world/trail/filtered",
            "/log/association"} <= entities


def test_viewer_stream_does_not_get_states_twice():
    logged = []
    bus = Bus()
    sink = RerunBusSink(bus, None, {}, log_fn=lambda topic, p, t: logged.append(topic))
    sink.start()
    try:
        bus.publish(ESTIMATE_TOPIC, estimate(), 0.0)
        bus.publish("telemetry/attitude", {"roll_deg": 1.0}, 0.0)
    finally:
        sink.close()
    assert logged == ["telemetry/attitude"]


CHILD = """
import signal, sys, time
from pathlib import Path
from skybridge.perception.session import die_with_parent
done = Path(sys.argv[1])
signal.signal(signal.SIGTERM, lambda *_: (done.write_text("clean"), sys.exit(0)))
die_with_parent(int(sys.argv[2]))
Path(sys.argv[3]).write_text("ready")
time.sleep(30)
"""
PARENT = """
import os, subprocess, sys, time
subprocess.Popen([sys.executable, "-c", sys.argv[1], sys.argv[2], str(os.getpid()), sys.argv[3]],
                 start_new_session=True)
time.sleep(30)
"""


def test_child_exits_cleanly_when_its_parent_is_killed(tmp_path):
    import signal
    import subprocess
    import sys

    done, ready = tmp_path / "done", tmp_path / "ready"
    parent = subprocess.Popen([sys.executable, "-c", PARENT, CHILD, str(done), str(ready)])
    try:
        assert wait_until(ready.exists, timeout=15.0), "child never started"
        parent.send_signal(signal.SIGKILL)
        parent.wait()
        assert wait_until(done.exists, timeout=5.0), "child outlived its parent"
        assert done.read_text() == "clean"
    finally:
        parent.kill()


def logged_entities(tmp_path, detail):
    """Entities (excluding static ones) from a live-style sink over a tracked run."""
    import rerun as rr
    import rerun.chunk as rrc

    path = tmp_path / f"detail_{detail}.rrd"
    rec = rr.RecordingStream("t", recording_id=str(detail))
    rec.save(path)
    pipeline = DepthPipeline(PARAMS, CAMERA)
    sink = RerunSink(pipeline, recording=rec, timeline="monotonic", detail=detail, log_mask=False,
                     log_depth=detail)
    sink.log_static()
    for f in frames():
        sink.log_result(pipeline.process(f))
    rec.flush()
    rec.disconnect()
    chunks = list(rrc.RrdReader(str(path)).stream())
    return ({c.entity_path for c in chunks if not c.is_static},
            {c.entity_path for c in chunks if c.is_static})


def test_live_view_sends_far_fewer_entities(tmp_path):
    full, _ = logged_entities(tmp_path, True)
    lean, static = logged_entities(tmp_path, False)
    assert len(lean) * 2 < len(full)
    assert lean == {"/world/filtered", "/world/trail/filtered", "/plots/position",
                    "/plots/velocity", "/log/association"}
    assert "/world/camera/background" in static


def test_background_can_be_thinned():
    pipeline = DepthPipeline(PARAMS, CAMERA)
    for f in frames(n_empty=5, n_drone=0):
        pipeline.process(f)
    full = RerunSink(pipeline).background_points()
    thin = RerunSink(pipeline, background_stride=4).background_points()
    assert len(full) == 64 * 48 and len(thin) == 16 * 12


def test_background_is_logged_even_if_its_frame_was_skipped():
    calls = []
    pipeline = DepthPipeline(PARAMS, CAMERA)
    sink = RerunSink(pipeline, detail=False, log_mask=False)
    sink._log = lambda path, archetype, static=False: calls.append(path)
    sink._set_time = lambda t, wall: None
    results = [pipeline.process(f) for f in frames()]
    sink.log_result(results[-1])  # the warm-up frames never reached the viewer
    assert calls.count("world/camera/background") == 1


def test_idle_live_view_clears_each_entity_only_once():
    calls = []
    pipeline = DepthPipeline(PARAMS, CAMERA)
    sink = RerunSink(pipeline, detail=False, log_mask=False)

    def record(path, archetype, static=False):
        sink._cleared.discard(path)
        calls.append(path)

    sink._log = record
    sink._set_time = lambda t, wall: None
    for f in frames(n_empty=8, n_drone=0):
        sink.log_result(pipeline.process(f))
    assert calls.count("world/filtered") == 1


def test_latest_worker_never_blocks_the_producer_and_keeps_the_newest():
    release, handled = threading.Event(), []

    def slow(item):
        release.wait(5.0)
        handled.append(item)

    worker = LatestWorker(slow)
    worker.start()
    try:
        started = time.perf_counter()
        for i in range(1000):
            worker.handle(RESULT_TOPIC, i, 0.0)
            if i == 0:
                time.sleep(0.05)  # let the worker pick up the first item and block
        assert time.perf_counter() - started < 0.5
        assert worker.skipped >= 900
    finally:
        release.set()
        assert wait_until(lambda: 999 in handled)
        worker.close()


def test_latest_worker_survives_a_failing_handler():
    handled = []

    def flaky(item):
        handled.append(item)
        if item == 1:
            raise RuntimeError("viewer went away")

    worker = LatestWorker(flaky)
    worker.start()
    try:
        worker.handle(RESULT_TOPIC, 1, 0.0)
        assert wait_until(lambda: handled == [1])
        worker.handle(RESULT_TOPIC, 2, 0.0)
        assert wait_until(lambda: handled == [1, 2])
    finally:
        worker.close()


def test_flight_recording_replays_through_the_offline_reader(tmp_path):
    meta = {"depth_intrinsics": CAMERA.intrinsics, "depth_scale_m_per_unit": CAMERA.depth_scale}
    recorder = DepthRecorder(tmp_path / "flight.rrd", meta, CAMERA.depth_scale)
    originals = frames(n_empty=2, n_drone=3, start=1000.0)
    for f in originals:
        result = PerceptionResult(f, DroneEstimate(f.t, f.seq, "warmup", False))
        recorder.handle(RESULT_TOPIC, result, f.t)
    recorder.close(skipped=7)

    source = RrdReplaySource(tmp_path / "flight.rrd")
    assert source.camera == CAMERA
    assert source.meta["frames_logged"] == {"depth": 5}
    assert source.meta["frames_dropped"] == {"depth": 7}
    replayed = [source.read() for _ in range(5)]
    for original, frame in zip(originals, replayed):
        np.testing.assert_array_equal(frame.depth, original.depth)
        assert frame.t == pytest.approx(original.t - 1000.0, abs=1e-6)
    assert source.read() is None and not source.alive


# ── Config ────────────────────────────────────────────────────────────────────
def test_shipped_config_loads():
    path = Path(__file__).parent.parent / "configs" / "perception.yaml"
    cfg = PerceptionConfig.from_yaml(path)
    assert cfg.camera.fps == 30 and cfg.pipeline.max_points == 3000


@pytest.mark.parametrize("raw", [
    {"nonsense": {}},
    {"pipeline": {"typo_key": 1}},
    {"pipeline": {"x_min": 2.0, "x_max": 1.0}},
    {"pipeline": {"bg_frames": 0}},
])
def test_bad_config_is_refused(raw):
    with pytest.raises(ConfigError):
        PerceptionConfig.from_dict(raw)
