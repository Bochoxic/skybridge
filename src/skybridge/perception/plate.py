"""The drone's yaw from a small vertical plate mounted on top of it.

The plate is a flat card standing upright on the drone. Seen from the camera
its horizontal cross-section is a line, and the direction of that line turns
with the drone. Measured on the full-resolution depth image around the
detected drone:

    1. pixels near the drone's depth, back-projected and moved to world
    2. points within ``radius`` of their median: the drone
    3. those above their median height, plus a margin: the plate (below is the frame)
    4. the long axis of the plate's points in the horizontal plane (PCA)

Convention: **0 = plate parallel to the image plane, + = drone yawed left**
(counter-clockwise seen from above), in [-90, 90). A flat plate looks the
same turned 180 degrees, so this is yaw modulo 180; the heading filter in
:mod:`skybridge.heading` restores the full circle from FC yaw.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .types import CameraModel


@dataclass(frozen=True)
class PlateParams:
    enabled: bool = True
    window: float = 0.15        # m, half-size of the image patch searched around the drone
    depth_gate: float = 0.15    # m, keep pixels this close in depth to the drone
    radius: float = 0.12        # m, keep points this close to their median: the drone
    split_margin: float = 0.005 # m above the median height where the plate starts; keeps
                                # out the frame's top face, which a downward camera sees
    min_points: int = 30        # in the drone, or no measurement
    min_plate_points: int = 15  # in the plate (its upper half), or no measurement


@dataclass(frozen=True, eq=False)
class PlateMeasurement:
    yaw_deg: float              # see the module docstring
    elong: float                # long-axis / short-axis variance: how clearly a line it is
    center: np.ndarray          # plate centroid, world
    axis: np.ndarray            # unit vector along the plate, world, horizontal
    half_width: float           # m, along the plate (5th-95th percentile)
    half_height: float          # m
    plate: np.ndarray           # plate points, world (for the viewer)
    body: np.ndarray            # the rest of the drone, world (for the viewer)

    def outline(self) -> np.ndarray:
        """The fitted plate as a closed rectangle, 5x3, world, for drawing."""
        up = np.array([0.0, 0.0, 1.0])
        w, h = self.half_width * self.axis, self.half_height * up
        return np.array([self.center + w + h, self.center + w - h, self.center - w - h,
                         self.center - w + h, self.center + w + h])


def fold(angle_deg: float) -> float:
    """Wrap an angle to [-90, 90): the plate's yaw is only known modulo 180."""
    return (angle_deg + 90.0) % 180.0 - 90.0


def measure_plate(
    depth: np.ndarray,
    camera: CameraModel,
    cam_R: np.ndarray,
    cam_t: np.ndarray,
    drone_world: np.ndarray,
    params: PlateParams,
) -> PlateMeasurement | None:
    """Measure the plate around `drone_world`, or None if it can't be seen well enough."""
    intr = camera.intrinsics
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]
    p = cam_R.T @ (np.asarray(drone_world, dtype=float) - cam_t)
    if not np.all(np.isfinite(p)) or p[2] < 0.1:
        return None
    u, v = fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy
    r = int(params.window * fx / p[2])
    h, w = depth.shape
    u0, u1 = max(int(u - r), 0), min(int(u + r) + 1, w)
    v0, v1 = max(int(v - r), 0), min(int(v + r) + 1, h)
    if r < 1 or u1 <= u0 or v1 <= v0:
        return None

    # Gate on the raw depth units, so only the drone's pixels are converted.
    raw = depth[v0:v1, u0:u1]
    lo = max((p[2] - params.depth_gate) / camera.depth_scale, 1.0)
    hi = (p[2] + params.depth_gate) / camera.depth_scale
    vv, uu = np.nonzero((raw >= lo) & (raw <= hi))
    if len(vv) < params.min_points:
        return None
    z = raw[vv, uu] * camera.depth_scale
    cam = np.column_stack(((uu + u0 - cx) / fx * z, (vv + v0 - cy) / fy * z, z))
    world = cam @ cam_R.T + cam_t
    world = world[np.linalg.norm(world - np.median(world, axis=0), axis=1) < params.radius]
    if len(world) < params.min_points:
        return None

    upper = world[:, 2] > np.median(world[:, 2]) + params.split_margin
    plate, body = world[upper], world[~upper]
    if len(plate) < params.min_plate_points:
        return None
    xy = plate[:, :2] - plate[:, :2].mean(axis=0)
    evals, evecs = np.linalg.eigh(xy.T @ xy / len(xy))
    major = evecs[:, -1]
    # Bearing of the long axis, clockwise from world +y (forward). Parallel to
    # the image is 90; turning left (counter-clockwise) by th gives 90 - th.
    bearing = np.degrees(np.arctan2(major[0], major[1]))
    w5, w95 = np.percentile(xy @ major, (5, 95))
    h5, h95 = np.percentile(plate[:, 2], (5, 95))
    return PlateMeasurement(
        yaw_deg=fold(90.0 - bearing),
        elong=float(evals[-1] / max(evals[0], 1e-12)),
        center=plate.mean(axis=0),
        axis=np.array([major[0], major[1], 0.0]),
        half_width=float(w95 - w5) / 2,
        half_height=float(h95 - h5) / 2,
        plate=plate,
        body=body,
    )
