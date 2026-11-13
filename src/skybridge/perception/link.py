"""Carry estimates from the perception process to the flight process over UDP.

UDP on loopback suits this link: sending never blocks, a lost or late estimate
is worthless anyway, and either process can die or restart without the other
noticing. ``time.monotonic()`` is system-wide on Linux, so capture times need
no conversion between the two processes.
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from collections.abc import Callable
from typing import Any

from ..bus import Bus
from ..state import Timestamped
from .types import DroneEstimate

log = logging.getLogger(__name__)

TOPIC = "perception/estimate"
MAX_DATAGRAM = 2048
STALE_AFTER = 0.5
# Learning the background blocks the loop for about half a second, once.
WARMUP_STALE_AFTER = 2.0


class EstimateSender:
    """Sends each estimate as one datagram. Can be used as a Bus subscriber."""

    def __init__(self, host: str, port: int) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self.sent = 0
        self.dropped = 0

    def send(self, estimate: DroneEstimate) -> None:
        try:
            self._sock.sendto(estimate.to_wire(), self._addr)
            self.sent += 1
        except OSError:
            self.dropped += 1

    def handle(self, topic: str, estimate: DroneEstimate, t: float) -> None:
        self.send(estimate)

    def close(self) -> None:
        self._sock.close()


class PerceptionLink:
    """Receives estimates on a background thread.

    The newest is available from :meth:`latest`, with its age measured from
    the moment the frame was captured, and each one is published on the Bus
    under ``perception/estimate``.
    """

    def __init__(
        self,
        host: str,
        port: int,
        bus: Bus | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._bus = bus
        self._clock = clock
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))  # fails now, loudly, if the port is taken
        self._sock.settimeout(0.1)
        self.port = self._sock.getsockname()[1]
        self._lock = threading.Lock()
        self._latest: Timestamped[DroneEstimate] | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.received = 0
        self.rejected = 0

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._run, name="skybridge-perception-link", daemon=True
            )
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._sock.close()

    def latest(self) -> Timestamped[DroneEstimate] | None:
        """The newest estimate, with its age pinned to now, or None before the first."""
        with self._lock:
            latest = self._latest
        return None if latest is None else latest.at(self._clock())

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._sock.recv(MAX_DATAGRAM)
            except TimeoutError:
                continue
            except OSError:
                return
            self.receive(data)

    def receive(self, data: bytes) -> None:
        """Handle one datagram. Older-than-newest and malformed ones are dropped."""
        try:
            estimate = DroneEstimate.from_wire(data)
        except ValueError:
            self.rejected += 1
            return
        with self._lock:
            if self._latest is not None and estimate.t <= self._latest.value.t:
                self.rejected += 1
                return
            self._latest = Timestamped(estimate, estimate.t)
        self.received += 1
        if self._bus is not None:
            self._bus.publish(TOPIC, estimate, estimate.t)


def vis_status(latest: Timestamped[DroneEstimate] | None, stale_after: float = STALE_AFTER) -> str:
    """One short phrase for the pilot's status line."""
    if latest is None:
        return "VIS waiting"
    e: Any = latest.value
    if e.status == "warmup" and not latest.is_stale(WARMUP_STALE_AFTER):
        return "VIS warmup"
    if latest.is_stale(stale_after):
        return f"VIS LOST {latest.age:.1f}s"
    if not e.valid:
        return f"VIS {e.status}"
    return f"VIS z={e.pos[2]:.2f}m v={e.vel[2]:+.1f} {e.status} {latest.age * 1000:.0f}ms"
