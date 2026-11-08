"""Perception settings: YAML in, validated dataclasses out.

Kept apart from :mod:`skybridge.config`, whose job is to mirror the flight
controller. This file describes the camera rig and the tracking pipeline.

:class:`PipelineParams` is the single source of defaults: the argparse flags
of ``drone_detect.py`` read theirs from it, and its field names match those
flags.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

import yaml

from ..config import ConfigError


@dataclass(frozen=True)
class PipelineParams:
    # Crop box, camera frame (x right, y down, z forward), metres
    x_min: float = -1.0
    x_max: float = 1.0
    y_min: float = -1.0
    y_max: float = 1.0
    z_min: float = 0.3
    z_max: float = 2.0
    # Static background
    bg_frames: int = 30
    bg_delta: float = 0.10
    bg_delta_rel: float = 0.02
    no_bg: bool = False
    # Camera pose -> world frame
    cam_height: float = 0.0
    cam_pitch: float = 0.0
    # Detection
    stride: int = 2
    eps: float = 0.05
    min_samples: int = 5
    min_points: int = 1
    max_points: int = 3000
    persist: int = 2
    gate: float = 0.5
    min_size: float = 0.0
    max_size: float = 0.4
    ambiguity_ratio: float = 0.5
    sparse_radius: float = 0.10
    sparse_min_points: int = 2
    sparse_noise: float = 3.0
    # Filter
    accel_std: float = 3.0
    meas_std: float = 0.02
    max_coast: float = 1.5
    blend_ms: float = 100.0

    def __post_init__(self) -> None:
        for lo, hi in (("x_min", "x_max"), ("y_min", "y_max"), ("z_min", "z_max")):
            if getattr(self, lo) >= getattr(self, hi):
                raise ConfigError(f"crop box: {lo} must be below {hi}")
        if self.bg_frames < 1:
            raise ConfigError("bg_frames must be at least 1")
        if self.stride < 1:
            raise ConfigError("stride must be at least 1")
        if self.max_points < 0:
            raise ConfigError("max_points must be >= 0 (0 disables the cap)")


@dataclass(frozen=True)
class CameraConfig:
    width: int = 848
    height: int = 480
    fps: int = 30
    color: bool = False


@dataclass(frozen=True)
class ViewerConfig:
    detail: bool = False        # every debug entity; too much for a live viewer
    depth: bool = False         # the live depth image
    background_stride: int = 4  # thin the one-time background by this, on top of pipeline.stride
    trail_seconds: float = 3.0
    arrow_scale: float = 0.3
    depth_stride: int = 2       # 2 = half resolution
    depth_hz: float = 10.0
    color_hz: float = 5.0
    memory_limit: str = "2GB"


@dataclass(frozen=True)
class LinkConfig:
    host: str = "127.0.0.1"
    port: int = 14660


@dataclass(frozen=True)
class PerceptionConfig:
    pipeline: PipelineParams = field(default_factory=PipelineParams)
    camera: CameraConfig = field(default_factory=CameraConfig)
    viewer: ViewerConfig = field(default_factory=ViewerConfig)
    link: LinkConfig = field(default_factory=LinkConfig)

    @classmethod
    def from_yaml(cls, path: str | Path) -> PerceptionConfig:
        raw = yaml.safe_load(Path(path).read_text())
        if not isinstance(raw, dict):
            raise ConfigError(f"{path}: expected a YAML mapping at the top level")
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> PerceptionConfig:
        sections = {f.name: f.default_factory for f in fields(cls)}
        if unknown := set(raw) - set(sections):
            raise ConfigError(f"unknown perception sections: {sorted(unknown)}")
        return cls(**{name: _build(make, name, raw.get(name) or {})
                      for name, make in sections.items()})


def _build(make: Any, name: str, raw: dict[str, Any]) -> Any:
    known = {f.name for f in fields(make)}
    if unknown := set(raw) - known:
        raise ConfigError(f"unknown {name} keys: {sorted(unknown)}")
    return make(**raw)
