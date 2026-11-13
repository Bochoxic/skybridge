"""Tracker behaviour across depth dropouts (src/skybridge/perception/tracker.py)."""
from types import SimpleNamespace

import numpy as np
import pytest

from skybridge.perception.geometry import camera_to_world
from skybridge.perception.tracker import Tracker

FPS = 30.0
ARGS = SimpleNamespace(
    min_points=8, eps=0.05, min_samples=5, no_bg=False,
    min_size=0.0, max_size=0.6, ambiguity_ratio=0.5,
    gate=0.3, persist=5, accel_std=3.0, meas_std=0.02, max_coast=0.5,
    sparse_radius=0.10, sparse_min_points=2, sparse_noise=3.0, blend_ms=0.0)


def blob(center):
    """A 5x5x2 grid of points, 1 cm apart: a small drone-sized cluster."""
    g = np.arange(5) * 0.01
    gx, gy, gz = np.meshgrid(g, g, g[:2], indexing="ij")
    pts = np.column_stack((gx.ravel(), gy.ravel(), gz.ravel()))
    return pts - pts.mean(axis=0) + center


def drone_at(frame, speed):
    """Camera-frame centroid of a drone flying along camera x at `speed` m/s."""
    return np.array([-0.5 + speed * frame / FPS, 0.0, 1.5])


def run(frames_visible, n_frames, speed, args=ARGS):
    tracker = Tracker(args, *camera_to_world(0.0, 0.0))
    out = []
    for f in range(n_frames):
        pts = blob(drone_at(f, speed)) if f in frames_visible else np.empty((0, 3))
        out.append(tracker.step(f / FPS, pts))
    return out


def test_new_track_needs_persist_frames():
    out = run(set(range(10)), 10, speed=0.6)
    detected = [o[5] for o in out]
    assert detected == [False] * 4 + [True] * 6
    assert [o[1] for o in out[:4]] == ["candidate"] * 4


def test_reacquired_at_prediction_after_short_dropout():
    # 1.5 m/s, 10 missing frames (0.33 s < max_coast): the drone moves 0.5 m,
    # beyond --gate from its last centroid but close to the KF prediction.
    visible = set(range(20)) | set(range(30, 40))
    out = run(visible, 40, speed=1.5)
    assert all(o[1] == "coasting" for o in out[20:30])
    assert np.linalg.norm(drone_at(30, 1.5) - drone_at(19, 1.5)) > ARGS.gate
    # Accepted on the first frame back: no new --persist streak
    assert all(o[5] for o in out[30:40])


def test_track_resets_after_max_coast():
    # 20 missing frames = 0.67 s > max_coast: a fresh track must persist again
    visible = set(range(10)) | set(range(30, 40))
    out = run(visible, 40, speed=0.3)
    assert [o[5] for o in out[30:34]] == [False] * 4
    assert out[34][5]


def test_cluster_far_from_prediction_is_not_taken():
    tracker = Tracker(ARGS, *camera_to_world(0.0, 0.0))
    for f in range(10):
        tracker.step(f / FPS, blob(drone_at(f, 0.6)))
    far = drone_at(10, 0.6) + np.array([0.0, 0.0, 0.8])  # 0.8 m off
    cluster, status, *_ , detected = tracker.step(10 / FPS, blob(far))
    assert cluster is None and status == "coasting" and not detected


def test_camera_world_roundtrip():
    R, t = camera_to_world(1.2, 20.0)
    tracker = Tracker(ARGS, R, t)
    p = np.array([0.3, -0.2, 1.7])
    assert tracker.to_camera(tracker.to_world(p)) == pytest.approx(p)


def tracked(args=ARGS, frames=10, speed=0.6):
    tracker = Tracker(args, *camera_to_world(0.0, 0.0))
    for f in range(frames):
        tracker.step(f / FPS, blob(drone_at(f, speed)))
    return tracker


def test_reason_outside_gate_reports_distance():
    tracker = tracked()
    far = drone_at(10, 0.6) + np.array([0.0, 0.0, 0.8])
    tracker.step(10 / FPS, blob(far))
    d = tracker.diag
    assert d["reason"] == "outside_gate"
    assert d["cand_dist"] == pytest.approx(0.8, abs=0.05) and d["gate"] == ARGS.gate
    assert d["since_update"] == pytest.approx(1 / FPS)


def sparse_points():
    # 8 points 8 cm apart from the prediction outwards: too sparse for DBSCAN
    return drone_at(10, 0.6) + np.arange(8)[:, None] * np.array([0.08, 0, 0])


def test_reason_dbscan_noise_with_points_near_prediction():
    from types import SimpleNamespace as NS
    tracker = tracked(NS(**{**vars(ARGS), "sparse_radius": 0.0}))
    _, status, *_ = tracker.step(10 / FPS, sparse_points())
    d = tracker.diag
    assert status == "coasting" and d["reason"] == "dbscan_noise" and d["near_pts"] > 0


def test_sparse_points_at_prediction_are_associated():
    tracker = tracked()
    _, status, *_, detected = tracker.step(10 / FPS, sparse_points())
    assert detected and status == "detected" and tracker.diag["reason"] == "ok_sparse"
    assert tracker.diag["cand_n"] == 2  # only the points within 10 cm


def test_sparse_needs_active_track():
    tracker = Tracker(ARGS, *camera_to_world(0.0, 0.0))
    _, status, *_, detected = tracker.step(0.0, sparse_points())
    assert not detected and tracker.diag["reason"] == "dbscan_noise"


def test_reason_no_points_and_lost_timeout():
    tracker = tracked()
    tracker.step(10 / FPS, np.empty((0, 3)))
    assert tracker.diag["reason"] == "no_points"
    _, status, *_ = tracker.step(10 / FPS + ARGS.max_coast + 0.1, np.empty((0, 3)))
    assert status == "lost" and tracker.diag["reason"] == "lost_timeout"


def test_blend_softens_reacquisition_jump():
    from types import SimpleNamespace as NS
    jumps = {}
    for blend in (0.0, 100.0):
        args = NS(**{**vars(ARGS), "blend_ms": blend})
        tracker = tracked(args, frames=20, speed=1.5)
        for f in range(20, 30):  # coast
            tracker.step(f / FPS, np.empty((0, 3)))
        before = tracker.output()
        tracker.step(30 / FPS, blob(drone_at(30, 1.5) + [0.0, 0.1, 0.0]))  # off-track
        jumps[blend] = np.linalg.norm(tracker.output() - before)
    assert jumps[100.0] < jumps[0.0]
