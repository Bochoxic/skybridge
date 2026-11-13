"""The live loop: frames in, results out on a Bus.

Topics published:

    perception/estimate   DroneEstimate      for the link and any other consumer
    perception/result     PerceptionResult   with depth and points, this process only

The estimate goes first, so viewer logging can never delay it. The Bus also
keeps a failing subscriber from taking the loop down.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from ..bus import Bus
from .pipeline import DepthPipeline
from .sources import FrameSource
from .types import PerceptionResult

ESTIMATE_TOPIC = "perception/estimate"
RESULT_TOPIC = "perception/result"


@dataclass
class RunStats:
    processed: int = 0
    status: str = ""
    process_ms: deque[float] = field(default_factory=lambda: deque(maxlen=300))
    latency_ms: deque[float] = field(default_factory=lambda: deque(maxlen=300))

    def summary(self, skipped: int) -> str:
        if not self.process_ms:
            return "no frames yet"
        proc = np.percentile(self.process_ms, [50, 99])
        late = np.percentile(self.latency_ms, [50, 99])
        return (f"{self.processed} frames, {skipped} skipped | process p50 {proc[0]:.0f} "
                f"p99 {proc[1]:.0f} ms | capture-to-output p50 {late[0]:.0f} "
                f"p99 {late[1]:.0f} ms | {self.status}")


def run(
    source: FrameSource,
    pipeline: DepthPipeline,
    bus: Bus,
    stop: threading.Event,
    *,
    on_result: Callable[[PerceptionResult, RunStats], None] | None = None,
    clock: Callable[[], float] = time.monotonic,
) -> RunStats:
    """Process frames until `stop` is set or the source ends."""
    stats = RunStats()
    while not stop.is_set() and source.alive:
        frame = source.read(timeout=0.2)
        if frame is None:
            continue
        started = clock()
        result = pipeline.process(frame)
        bus.publish(ESTIMATE_TOPIC, result.estimate, frame.t)
        finished = clock()
        bus.publish(RESULT_TOPIC, result, frame.t)

        stats.processed += 1
        stats.status = result.estimate.status
        stats.process_ms.append((finished - started) * 1000.0)
        stats.latency_ms.append((finished - frame.t) * 1000.0)
        if on_result is not None:
            on_result(result, stats)
    return stats
