"""Pixels to 3D points, and the camera to world transform.

Frames:
    camera  RealSense optical frame: x right, y down, z forward (depth).
    world   x right, y forward (horizontal), z up; origin on the floor
            straight below the camera.
"""

from __future__ import annotations

import numpy as np


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
