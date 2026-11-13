"""Where depth frames come from: the camera, a recording or a test fixture.

Kept behind a tiny interface, like :mod:`skybridge.rc.source`, so the runner
never knows which one it is reading. ``pyrealsense2`` and ``rerun`` are only
imported by the sources that need them.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections.abc import Callable, Iterable, Iterator
from itertools import islice
from pathlib import Path
from typing import Any, Generic, Protocol, TypeVar

import numpy as np

from .config import CameraConfig
from .types import CameraModel, DepthFrame

log = logging.getLogger(__name__)

T = TypeVar("T")

# A timestamp further than this from "now" cannot be a capture time: it is
# on some other clock (device hardware time, for instance).
MAX_CAPTURE_AGE = 1.0
MIN_CAPTURE_AGE = -0.1  # hardware and host clocks disagree slightly


class FrameSource(Protocol):
    camera: CameraModel
    meta: dict[str, Any]        # stored in the sidecar of a flight recording

    @property
    def alive(self) -> bool:
        """False once no more frames will ever arrive."""
        ...

    @property
    def skipped(self) -> int:
        """Frames thrown away because a newer one arrived first."""
        ...

    def read(self, timeout: float = 0.2) -> DepthFrame | None:
        """The newest unread frame, or None if none arrived within `timeout`."""
        ...

    def close(self) -> None: ...


class LatestSlot(Generic[T]):
    """Holds one item. A new item replaces an unread one, so readers never lag."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._item: T | None = None
        self.overwritten = 0

    def put(self, item: T) -> None:
        with self._cond:
            if self._item is not None:
                self.overwritten += 1
            self._item = item
            self._cond.notify()

    def get(self, timeout: float) -> T | None:
        with self._cond:
            if self._item is None:
                self._cond.wait(timeout)
            item, self._item = self._item, None
            return item


def capture_monotonic(timestamp_ms: float, mono_now: float, wall_now: float) -> float | None:
    """Convert a Unix-epoch capture timestamp to the ``time.monotonic()`` base.

    Returns None if the timestamp isn't plausibly a recent Unix time.
    """
    age = wall_now - timestamp_ms / 1000.0
    if not MIN_CAPTURE_AGE <= age <= MAX_CAPTURE_AGE:
        return None
    return mono_now - age


class RealSenseSource:
    """Live depth from a RealSense. The newest frame wins, so latency stays bounded."""

    def __init__(self, config: CameraConfig) -> None:
        import pyrealsense2 as rs

        from ..optitrack.realsense_record import intrinsics_dict, option_or_none

        self._rs = rs
        self._slot: LatestSlot[DepthFrame] = LatestSlot()
        self._color: np.ndarray | None = None
        self._last_t = float("-inf")
        self._closed = False
        self._warned_clock = False
        self.timestamp_domain: str | None = None

        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.depth, config.width, config.height,
                                rs.format.z16, config.fps)
        if config.color:
            rs_config.enable_stream(rs.stream.color, config.width, config.height,
                                    rs.format.bgr8, config.fps)

        self._pipeline = rs.pipeline()
        profile = self._pipeline.start(rs_config, self._on_frames)
        device = profile.get_device()
        for sensor in device.query_sensors():
            if sensor.supports(rs.option.global_time_enabled):
                sensor.set_option(rs.option.global_time_enabled, 1)

        depth_sensor = device.first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        intrinsics = intrinsics_dict(profile.get_stream(rs.stream.depth))
        self.camera = CameraModel(intrinsics, depth_scale)
        self.meta = {
            "device": device.get_info(rs.camera_info.name),
            "serial": device.get_info(rs.camera_info.serial_number),
            "firmware": device.get_info(rs.camera_info.firmware_version),
            "resolution": [config.width, config.height],
            "fps": config.fps,
            "depth_scale_m_per_unit": depth_scale,
            "depth_intrinsics": intrinsics,
            "depth_options": {
                "emitter_enabled": option_or_none(depth_sensor, rs.option.emitter_enabled),
                "laser_power": option_or_none(depth_sensor, rs.option.laser_power),
                "exposure": option_or_none(depth_sensor, rs.option.exposure),
                "gain": option_or_none(depth_sensor, rs.option.gain),
            },
        }

    @property
    def alive(self) -> bool:
        return not self._closed

    @property
    def skipped(self) -> int:
        return self._slot.overwritten

    def read(self, timeout: float = 0.2) -> DepthFrame | None:
        return self._slot.get(timeout)

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._pipeline.stop()

    # Runs on a librealsense thread. Copy out and return: holding on to frames
    # exhausts the SDK's frame pool.
    def _on_frames(self, item: Any) -> None:
        try:
            frames = list(item.as_frameset()) if item.is_frameset() else [item]
            for frame in frames:
                stream = frame.get_profile().stream_type()
                if stream == self._rs.stream.color:
                    self._color = np.asanyarray(frame.get_data()).copy()
                elif stream == self._rs.stream.depth:
                    self._on_depth(frame)
        except Exception:
            log.exception("failed to read a RealSense frame")

    def _on_depth(self, frame: Any) -> None:
        mono, wall = time.monotonic(), time.time()
        domain = str(frame.get_frame_timestamp_domain())
        if domain != self.timestamp_domain:
            self.timestamp_domain = domain
            log.info("depth timestamp domain: %s", domain)
            if "global_time" not in domain:
                log.warning("depth timestamps are not hardware-derived (%s): "
                            "velocity estimates will be noisier", domain)

        t = capture_monotonic(frame.get_timestamp(), mono, wall)
        if t is None:
            if not self._warned_clock:
                self._warned_clock = True
                log.warning("implausible depth timestamp (%s): using arrival time", domain)
            t = mono
        if t <= self._last_t:  # the filter needs time to move forward
            return
        self._last_t = t

        self._slot.put(DepthFrame(
            depth=np.asanyarray(frame.get_data()).copy(),
            t=t,
            wall=wall - (mono - t),
            seq=frame.get_frame_number(),
            color=self._color,
        ))


def iter_depth_frames(rrd_path: str | Path) -> Iterator[tuple[float, float, np.ndarray]]:
    """Yield (wall_time_s, sensor_time_s, depth uint16 HxW) for every depth frame."""
    import rerun.chunk as rrc

    for chunk in rrc.RrdReader(str(rrd_path)).stream():
        if chunk.entity_path != "/camera/depth" or chunk.is_static:
            continue
        rb = chunk.to_record_batch()
        wall = rb.column("wall_time").cast("int64").to_numpy() / 1e9
        sensor = rb.column("sensor_time").cast("int64").to_numpy() / 1e9
        buffers = rb.column("DepthImage:buffer").to_pylist()
        formats = rb.column("DepthImage:format").to_pylist()
        for i in range(rb.num_rows):
            if not buffers[i]:
                continue
            fmt = formats[i][0]
            depth = np.frombuffer(bytes(buffers[i][0]), dtype=np.uint16)
            yield wall[i], sensor[i], depth.reshape(fmt["height"], fmt["width"])


class RrdReplaySource:
    """Frames from a ``.rrd`` recording with its JSON sidecar.

    By default every frame is returned, as fast as it can be read, with its
    recorded time. With ``realtime=True`` the recording plays like a camera:
    frames are stamped on the ``time.monotonic()`` base and any that the
    reader is too slow for are skipped. Decoding a ``.rrd`` is slower than
    30 fps, so realtime mode loads the frames into memory first.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        realtime: bool = False,
        max_frames: int = 0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        path = Path(path)
        self.meta: dict[str, Any] = json.loads(path.with_suffix(".json").read_text())
        self.camera = CameraModel.from_meta(self.meta)
        self.skipped = 0
        self._alive = True
        self._realtime = realtime
        self._clock = clock
        self._t0: float | None = None
        self._next = 0
        frames = islice(iter_depth_frames(path), max_frames or None)
        self._iter = iter(frames)
        self._loaded = list(frames) if realtime else []

    @property
    def alive(self) -> bool:
        return self._alive

    def read(self, timeout: float = 0.2) -> DepthFrame | None:
        return self._read_realtime(timeout) if self._realtime else self._read_all()

    def close(self) -> None:
        self._alive = False

    def _read_all(self) -> DepthFrame | None:
        item = next(self._iter, None)
        if item is None:
            self._alive = False
            return None
        wall, sensor, depth = item
        self._next += 1
        return DepthFrame(depth=depth, t=sensor, wall=wall, seq=self._next - 1)

    def _read_realtime(self, timeout: float) -> DepthFrame | None:
        frames = self._loaded
        if self._next >= len(frames):
            self._alive = False
            return None
        start = frames[0][1]
        if self._t0 is None:
            self._t0 = self._clock()

        wait = (frames[self._next][1] - start) - (self._clock() - self._t0)
        if wait > timeout:
            time.sleep(timeout)
            return None
        if wait > 0:
            time.sleep(wait)

        elapsed = self._clock() - self._t0
        index = self._next
        while index + 1 < len(frames) and frames[index + 1][1] - start <= elapsed:
            index += 1
        self.skipped += index - self._next
        self._next = index + 1

        _, sensor, depth = frames[index]
        t = self._t0 + (sensor - start)
        return DepthFrame(depth=depth, t=t, wall=time.time() - (self._clock() - t), seq=index)


class FakeFrameSource:
    """Plays a list of frames, for tests."""

    def __init__(self, frames: Iterable[DepthFrame], camera: CameraModel) -> None:
        self.camera = camera
        self.meta: dict[str, Any] = {}
        self.skipped = 0
        self.closed = False
        self._iter = iter(frames)
        self._alive = True

    @property
    def alive(self) -> bool:
        return self._alive

    def read(self, timeout: float = 0.2) -> DepthFrame | None:
        frame = next(self._iter, None)
        if frame is None:
            self._alive = False
        return frame

    def close(self) -> None:
        self.closed = True
        self._alive = False
