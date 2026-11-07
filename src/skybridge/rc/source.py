"""RC input sources: where stick and switch positions come from.

Kept behind a tiny interface, like :mod:`skybridge.transport`, so the
passthrough never knows whether it is reading a USB joystick, a recorded
log or a test fixture.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class RcFrame:
    """One coherent snapshot of the radio's outputs.

    ``axes`` are raw joystick values in -32767..32767 (EdgeTX mixer
    outputs, -100%..+100%); ``buttons`` are 0/1. ``received`` is the
    monotonic instant of the most recent change.
    """

    axes: tuple[int, ...]
    buttons: tuple[int, ...] = ()
    received: float = 0.0


class RcSource(Protocol):
    """Anything that yields RC frames."""

    @property
    def alive(self) -> bool:
        """False once the device is gone (unplugged, read error)."""
        ...

    def latest(self) -> RcFrame | None:
        """The current state, or None until it is fully known."""
        ...

    def close(self) -> None: ...


class FakeRcSource:
    """In-memory source for tests. Set the state with :meth:`set`."""

    def __init__(self) -> None:
        self.alive = True
        self.closed = False
        self._frame: RcFrame | None = None

    def set(
        self, axes: tuple[int, ...], buttons: tuple[int, ...] = (), at: float = 0.0
    ) -> None:
        self._frame = RcFrame(tuple(axes), tuple(buttons), at)

    def latest(self) -> RcFrame | None:
        return self._frame

    def close(self) -> None:
        self.closed = True
        self.alive = False
