"""A minimal publish/subscribe hub: the single place data leaves the loop.

Producers (the RC source, the Bridge, later the RealSense) publish
timestamped messages under slash-separated topics; consumers subscribe
by prefix. This is the hook for recording -- a recorder subscribes to
``""`` (everything) or ``"telemetry/"`` and writes to disk -- without any
producer knowing it exists.

Topics in use:

    rc/input              RcFrame         every change on the radio
    rc/sent               tuple[int, ...] the 16 ticks of every CRSF frame sent
    telemetry/<name>      dict            every decoded telemetry frame,
                                          name = attitude, battery, link_stats, ...
    perception/estimate   DroneEstimate   every tracked depth frame
    perception/result     PerceptionResult  depth and points, perception process only

Timestamps are ``time.monotonic()`` seconds (the Bridge's clock), the
same base as :class:`skybridge.state.Timestamped`.

**Callbacks run synchronously on the producer's thread** -- for
``rc/sent`` that is the RC thread, whose 20ms tick must not slip. A
subscriber must not block: anything slow (disk, network) belongs on its
own thread behind a queue. A subscriber that raises is logged and kept;
it can never take down the RC loop.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from typing import Any

log = logging.getLogger(__name__)

Callback = Callable[[str, Any, float], None]


class Bus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._subs: tuple[tuple[str, Callback], ...] = ()

    def subscribe(self, prefix: str, callback: Callback) -> Callable[[], None]:
        """Call `callback(topic, payload, t)` for topics starting with `prefix`.

        Returns a function that removes the subscription.
        """
        entry = (prefix, callback)
        with self._lock:
            self._subs = self._subs + (entry,)

        def unsubscribe() -> None:
            with self._lock:
                self._subs = tuple(s for s in self._subs if s is not entry)

        return unsubscribe

    def publish(self, topic: str, payload: Any, t: float | None = None) -> None:
        if t is None:
            t = time.monotonic()
        for prefix, callback in self._subs:  # immutable snapshot, no lock needed
            if topic.startswith(prefix):
                try:
                    callback(topic, payload, t)
                except Exception:
                    log.exception("bus subscriber failed on %r", topic)
