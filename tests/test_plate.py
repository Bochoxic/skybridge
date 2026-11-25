"""Plate yaw from a rendered depth image of a drone with an upright plate on top."""

import numpy as np
import pytest

from skybridge.perception import CameraModel, DepthFrame
from skybridge.perception.config import PipelineParams
from skybridge.perception.geometry import camera_to_world
from skybridge.perception.pipeline import DepthPipeline
from skybridge.perception.plate import PlateParams, fold, measure_plate

CAM = CameraModel({"width": 848, "height": 480, "fx": 433.0, "fy": 433.0,
                   "ppx": 424.0, "ppy": 240.0}, 0.001)
STEP = 0.0015  # m between sampled surface points: finer than a pixel at 1 m


def grid(a, b):
    """Points filling the rectangle spanned by vectors a and b around the origin."""
    s = np.arange(-0.5, 0.5 + 1e-9, STEP / max(np.linalg.norm(a), np.linalg.norm(b)))
    i, j = np.meshgrid(s, s)
    return i.reshape(-1, 1) * a + j.reshape(-1, 1) * b


def drone(center, yaw_deg):
    """World points on a 7x7x1.5 cm body and a 6x5 cm plate standing on it.

    yaw 0: the plate runs along world x, parallel to the image of a level camera.
    Positive yaw turns the drone counter-clockwise seen from above (left).
    """
    c = np.asarray(center, float)
    th = np.radians(yaw_deg)
    ax = np.array([np.cos(th), np.sin(th), 0.0])     # along the plate (drone's lateral axis)
    fw = np.array([-np.sin(th), np.cos(th), 0.0])    # drone's nose
    up = np.array([0.0, 0.0, 1.0])
    L, H = 0.07, 0.015
    faces = [c + up * H / 2 + grid(ax * L, fw * L), c - up * H / 2 + grid(ax * L, fw * L)]
    for d, side in ((ax, fw), (fw, ax)):
        for sign in (1, -1):
            faces.append(c + sign * d * L / 2 + grid(side * L, up * H))
    plate_c = c + up * (H / 2 + 0.025)
    for sign in (1, -1):
        faces.append(plate_c + sign * fw * 0.001 + grid(ax * 0.06, up * 0.05))
    return np.concatenate(faces)


def render(points, R, t, cam=CAM):
    """Depth image (uint16 mm) of world points, nearest surface wins."""
    intr = cam.intrinsics
    p = (points - t) @ R
    p = p[p[:, 2] > 0.05]
    u = np.round(intr["fx"] * p[:, 0] / p[:, 2] + intr["ppx"]).astype(int)
    v = np.round(intr["fy"] * p[:, 1] / p[:, 2] + intr["ppy"]).astype(int)
    ok = (u >= 0) & (u < intr["width"]) & (v >= 0) & (v < intr["height"])
    zbuf = np.full((intr["height"], intr["width"]), np.inf)
    np.minimum.at(zbuf, (v[ok], u[ok]), p[ok, 2])
    return np.where(np.isfinite(zbuf), np.round(zbuf / cam.depth_scale), 0).astype(np.uint16)


def measure(yaw, center=(0.1, 1.0, 0.3), cam_height=0.3, cam_pitch=0.0, **kw):
    R, t = camera_to_world(cam_height, cam_pitch)
    depth = render(drone(center, yaw), R, t)
    return measure_plate(depth, CAM, R, t, np.array(center), PlateParams(**kw))


@pytest.mark.parametrize("yaw", [-50, -20, 0, 20, 50])
def test_yaw_with_the_camera_level_with_the_drone(yaw):
    m = measure(yaw)
    assert m is not None
    assert m.yaw_deg == pytest.approx(yaw, abs=3)


@pytest.mark.parametrize("yaw", [-40, 0, 40])
def test_yaw_from_a_camera_looking_down(yaw):
    # The world frame absorbs the camera's pose: same yaw from above as level.
    m = measure(yaw, cam_height=0.8, cam_pitch=25.0)
    assert m is not None
    assert m.yaw_deg == pytest.approx(yaw, abs=4)


def test_left_is_positive():
    # Turning left moves the plate's right end (as the camera sees it) away
    # from the camera: its far end is on the right.
    m = measure(30)
    right_end = m.center + m.axis * m.half_width * np.sign(m.axis[0])
    assert right_end[1] > m.center[1]
    assert m.yaw_deg > 0


def test_measurement_describes_the_plate():
    m = measure(0)
    assert m.half_width == pytest.approx(0.03, abs=0.006)
    assert m.center[2] > 0.3                       # above the body's centre
    assert m.elong > 20                            # a clear line
    outline = m.outline()
    assert outline.shape == (5, 3) and np.allclose(outline[0], outline[-1])
    assert len(m.plate) >= 15 and len(m.body) > 0


def test_no_measurement_without_the_drone():
    R, t = camera_to_world(0.3, 0.0)
    empty = np.zeros((480, 848), np.uint16)
    assert measure_plate(empty, CAM, R, t, np.array([0.1, 1.0, 0.3]), PlateParams()) is None


def test_no_measurement_behind_the_camera_or_outside_the_image():
    R, t = camera_to_world(0.3, 0.0)
    depth = render(drone((0.1, 1.0, 0.3), 0), R, t)
    for where in ([0.1, -1.0, 0.3], [5.0, 1.0, 0.3]):
        assert measure_plate(depth, CAM, R, t, np.array(where), PlateParams()) is None


def test_too_few_points_is_no_measurement():
    assert measure(0, min_points=100_000) is None


def test_fold():
    assert fold(0) == 0 and fold(89) == 89 and fold(90) == -90
    assert fold(120) == -60 and fold(-100) == 80 and fold(270) == -90


def test_pipeline_puts_the_plate_on_detected_frames():
    R, t = camera_to_world(0.3, 0.0)
    pipeline = DepthPipeline(PipelineParams(bg_frames=2, stride=2, cam_height=0.3,
                                            min_points=8, persist=2), CAM)
    background = np.zeros((480, 848), np.uint16)
    frames = [DepthFrame(background, i / 30, seq=i) for i in range(2)]
    frames += [DepthFrame(render(drone((0.1, 1.0, 0.3), 25), R, t), (2 + i) / 30, seq=2 + i)
               for i in range(4)]
    results = [pipeline.process(f) for f in frames]
    last = results[-1]
    assert last.detected and last.plate is not None
    assert last.estimate.plate_yaw == pytest.approx(25, abs=3)
    assert last.estimate.plate_elong == pytest.approx(last.plate.elong)
    assert results[2].estimate.plate_yaw is None   # a candidate, not yet detected


def test_disabled_plate_is_not_measured():
    R, t = camera_to_world(0.3, 0.0)
    pipeline = DepthPipeline(PipelineParams(bg_frames=1, stride=2, cam_height=0.3,
                                            min_points=8, persist=1), CAM,
                             PlateParams(enabled=False))
    pipeline.process(DepthFrame(np.zeros((480, 848), np.uint16), 0.0))
    res = pipeline.process(DepthFrame(render(drone((0.1, 1.0, 0.3), 0), R, t), 0.1, seq=1))
    assert res.detected and res.plate is None and res.estimate.plate_yaw is None
