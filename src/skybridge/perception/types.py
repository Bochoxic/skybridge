"""Data that moves through perception.

:class:`DroneEstimate` is the output contract: small, immutable and safe to
send across threads and processes. Everything bulky (depth, points,
clusters) stays in :class:`PerceptionResult`, inside the perception process.

Time ``t`` is the frame's capture time. Live it is on the ``time.monotonic()``
base shared with the Bus and the Bridge; replayed from a recording it is
``sensor_time``.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

Vec3 = tuple[float, float, float]

WIRE_VERSION = 1


@dataclass(frozen=True)
class CameraModel:
    """Depth intrinsics (``width, height, fx, fy, ppx, ppy``) and depth scale."""

    intrinsics: dict[str, Any]
    depth_scale: float

    @classmethod
    def from_meta(cls, meta: dict[str, Any]) -> CameraModel:
        """Build from a recording's JSON sidecar."""
        return cls(meta["depth_intrinsics"], meta["depth_scale_m_per_unit"])


@dataclass(frozen=True, eq=False)
class DepthFrame:
    depth: np.ndarray           # uint16, HxW
    t: float                    # capture time, seconds
    wall: float = 0.0           # Unix time, seconds
    seq: int = 0
    color: np.ndarray | None = None  # BGR, HxWx3, only with the color stream on


@dataclass(frozen=True)
class DroneEstimate:
    """Where the drone is, in the world frame (x right, y forward, z up, metres)."""

    t: float
    seq: int
    status: str                 # warmup|none|ambiguous|candidate|detected|coasting|lost
    valid: bool                 # a confirmed track exists
    pos: Vec3 | None = None     # smoothed output position at time t
    vel: Vec3 | None = None     # m/s
    pos_std: Vec3 | None = None # 1-sigma position uncertainty from the filter
    since_update: float | None = None  # seconds since a detection updated the filter
    reason: str = ""            # why this frame was or wasn't associated

    def at(self, now: float, max_ahead: float = 0.2) -> Vec3 | None:
        """Position projected to `now` at constant velocity, or None if invalid.

        The projection is limited to `max_ahead` seconds, so a stale estimate
        cannot run away. It never goes backwards in time.
        """
        if not self.valid or self.pos is None or self.vel is None:
            return None
        ahead = min(max(now - self.t, 0.0), max_ahead)
        return (
            self.pos[0] + self.vel[0] * ahead,
            self.pos[1] + self.vel[1] * ahead,
            self.pos[2] + self.vel[2] * ahead,
        )

    def to_wire(self) -> bytes:
        return json.dumps({"v": WIRE_VERSION, **asdict(self)}).encode()

    @classmethod
    def from_wire(cls, data: bytes) -> DroneEstimate:
        """Parse a datagram. Raises ValueError on anything malformed."""
        try:
            raw = json.loads(data)
            if raw.pop("v") != WIRE_VERSION:
                raise ValueError("unsupported wire version")
            for key in ("pos", "vel", "pos_std"):
                if raw.get(key) is not None:
                    raw[key] = tuple(float(x) for x in raw[key])
            return cls(**raw)
        except (KeyError, TypeError, AttributeError) as exc:
            raise ValueError(f"malformed estimate: {exc}") from exc


@dataclass(frozen=True, eq=False)
class PerceptionResult:
    """Everything one frame produced: the estimate plus debug data for the viewer."""

    frame: DepthFrame
    estimate: DroneEstimate
    warmup: bool = False
    background_ready: bool = False      # the warm-up finished on this frame
    foreground: np.ndarray | None = None  # flat bool mask over the strided grid
    cropped: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    cluster: np.ndarray | None = None   # associated points, camera frame
    n_clusters: int = 0
    centroid: np.ndarray | None = None  # of the cluster, camera frame
    world_pos: np.ndarray | None = None # of the cluster, world frame
    detected: bool = False
    prediction: np.ndarray | None = None  # where the tracker searched, world frame
    filtered: np.ndarray | None = None  # filter position after this frame, world
    velocity: np.ndarray | None = None  # filter velocity after this frame, world
    output: np.ndarray | None = None    # smoothed position after this frame, world
    diag: dict[str, Any] = field(default_factory=dict)  # see detector.REASONS
