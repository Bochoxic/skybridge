"""Static depth background, learned from the first frames of a run."""

from __future__ import annotations

import warnings

import numpy as np


class DepthBackground:
    """Per-pixel static depth background from warm-up frames, on the strided grid."""

    def __init__(self, depth_scale, stride, delta, delta_rel):
        self.scale = depth_scale
        self.stride = stride
        self.delta = delta
        self.delta_rel = delta_rel
        self.frames = []
        self.z = None       # metres, NaN where the background has no depth
        self.thresh = None

    def add(self, depth):
        self.frames.append(depth[::self.stride, ::self.stride].copy())

    def finalize(self):
        stack = np.stack(self.frames).astype(np.float32)
        stack[stack == 0] = np.nan
        n_valid = np.count_nonzero(~np.isnan(stack), axis=0)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN pixels
            z = np.nanmedian(stack, axis=0) * self.scale
        # No reliable background where most warm-up frames had no depth
        z[2 * n_valid < len(self.frames)] = np.nan
        self.z = z
        # Depth noise grows roughly with z^2, so widen the threshold far away
        self.thresh = np.maximum(self.delta, self.delta_rel * z ** 2)
        self.frames = []

    def foreground(self, depth):
        """Flat bool mask: valid pixels in front of the static background."""
        z = depth[::self.stride, ::self.stride] * self.scale
        no_bg = np.isnan(self.z)
        with np.errstate(invalid="ignore"):
            closer = z < self.z - self.thresh  # False where background is NaN
        # One-sided: pixels now *farther* than the background never count
        return ((z > 0) & (closer | no_bg)).ravel()
