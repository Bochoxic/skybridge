"""Read an EdgeTX radio in USB Joystick mode via Linux joydev.

The radio appears as ``/dev/input/jsN`` (world-readable, no root and no
extra dependencies). Each change arrives as an 8-byte ``js_event``. On
open the kernel replays every axis and button with the INIT flag set,
which is how the starting state becomes known.

Two properties shape this module:

* **Nothing is reported until every axis has been initialised.** An
  axis that defaults to 0 reads as mid-stick -- on the throttle channel
  that would be 50% throttle.
* **Events only arrive on change**, so a quiet device is not a dead one
  and event age says nothing about liveness. An unplug does surface, as a
  read error, which flips :attr:`JoystickSource.alive`.
"""

from __future__ import annotations

import array
import fcntl
import os
import select
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from ..bus import Bus
from .source import RcFrame

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

_EVENT = struct.Struct("IhBB")  # u32 time_ms, s16 value, u8 type, u8 number

# ioctl request codes from <linux/joystick.h>
_JSIOCGAXES = 0x80016A11
_JSIOCGBUTTONS = 0x80016A12


def _jsiocgname(length: int) -> int:
    return 0x80006A13 + (length << 16)


@dataclass(frozen=True)
class JsEvent:
    time_ms: int
    value: int
    type: int
    number: int

    @property
    def is_axis(self) -> bool:
        return bool(self.type & JS_EVENT_AXIS)

    @property
    def is_button(self) -> bool:
        return bool(self.type & JS_EVENT_BUTTON)

    @property
    def is_init(self) -> bool:
        return bool(self.type & JS_EVENT_INIT)


def parse_js_events(buf: bytearray) -> list[JsEvent]:
    """Decode every complete event in `buf`, leaving a partial tail in place."""
    n = len(buf) // _EVENT.size * _EVENT.size
    events = [JsEvent(*fields) for fields in _EVENT.iter_unpack(bytes(buf[:n]))]
    del buf[:n]
    return events


class JoystickSource:
    """Background reader for a joydev device; an :class:`RcSource`.

    Publishes ``rc/input`` (an :class:`RcFrame`) on `bus` for every change
    once the state is fully known.
    """

    def __init__(
        self,
        device: str = "/dev/input/js0",
        *,
        bus: Bus | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.device = device
        self._bus = bus
        self._clock = clock
        try:
            self._fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
        except FileNotFoundError:
            raise FileNotFoundError(
                f"{device} not found: is the radio plugged in, and was "
                f"'USB Joystick' selected on its screen?"
            ) from None

        self.name = _ioctl_name(self._fd)
        self.num_axes = _ioctl_u8(self._fd, _JSIOCGAXES)
        self.num_buttons = _ioctl_u8(self._fd, _JSIOCGBUTTONS)

        self._lock = threading.Lock()
        self._axes = [0] * self.num_axes
        self._buttons = [0] * self.num_buttons
        self._axes_seen: set[int] = set()
        self._frame: RcFrame | None = None
        self._alive = True
        self.error: OSError | None = None

        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="skybridge-joystick", daemon=True
        )
        self._thread.start()

    # ── RcSource ──────────────────────────────────────────────────────────────
    @property
    def alive(self) -> bool:
        with self._lock:
            return self._alive

    def latest(self) -> RcFrame | None:
        with self._lock:
            return self._frame

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            os.close(self._fd)
        except OSError:
            pass
        with self._lock:
            self._alive = False

    def __enter__(self) -> JoystickSource:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def __repr__(self) -> str:
        return (
            f"JoystickSource({self.device!r}, {self.name!r}, "
            f"axes={self.num_axes}, buttons={self.num_buttons})"
        )

    # ── Thread body ───────────────────────────────────────────────────────────
    def _run(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.1)
                if not ready:
                    continue
                data = os.read(self._fd, _EVENT.size * 64)
            except BlockingIOError:
                continue
            except OSError as exc:  # ENODEV on unplug
                self._die(exc)
                return
            if not data:
                self._die(OSError("device closed"))
                return
            buf.extend(data)
            self._apply(parse_js_events(buf))

    def _apply(self, events: list[JsEvent]) -> None:
        if not events:
            return
        now = self._clock()
        with self._lock:
            for ev in events:
                if ev.is_axis and ev.number < self.num_axes:
                    self._axes[ev.number] = ev.value
                    self._axes_seen.add(ev.number)
                elif ev.is_button and ev.number < self.num_buttons:
                    self._buttons[ev.number] = ev.value
            if len(self._axes_seen) < self.num_axes:
                return
            frame = RcFrame(tuple(self._axes), tuple(self._buttons), now)
            self._frame = frame
        if self._bus is not None:
            self._bus.publish("rc/input", frame, now)

    def _die(self, exc: OSError) -> None:
        with self._lock:
            self._alive = False
            self.error = exc


def _ioctl_u8(fd: int, request: int) -> int:
    out = array.array("B", [0])
    fcntl.ioctl(fd, request, out)
    return out[0]


def _ioctl_name(fd: int, length: int = 128) -> str:
    out = array.array("B", [0] * length)
    fcntl.ioctl(fd, _jsiocgname(length), out)
    return out.tobytes().split(b"\0", 1)[0].decode(errors="replace")
