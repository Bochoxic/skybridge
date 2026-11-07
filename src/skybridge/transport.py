"""Serial transport.

Kept behind a tiny interface so the Bridge only ever calls `write()` and
`read_available()`. That containment is deliberate: if the single-thread
design ever needs to become a separate reader thread, this is the only
file that changes.
"""

from __future__ import annotations

from typing import Protocol

import serial


class Transport(Protocol):
    """Minimal byte pipe."""

    def write(self, data: bytes) -> None: ...

    def read_available(self, max_bytes: int = 512) -> bytes: ...

    def close(self) -> None: ...


class SerialTransport:
    """pyserial-backed transport, opened non-blocking."""

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self._ser = serial.Serial(port, baud, timeout=0)

    def write(self, data: bytes) -> None:
        self._ser.write(data)

    def read_available(self, max_bytes: int = 512) -> bytes:
        return self._ser.read(max_bytes)

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()

    def __repr__(self) -> str:
        return f"SerialTransport({self.port!r}, {self.baud})"


class LoopbackTransport:
    """In-memory transport for tests.

    Everything written is recorded; `feed()` queues bytes to be read back.
    """

    def __init__(self) -> None:
        self.written = bytearray()
        self._to_read = bytearray()
        self.closed = False

    def write(self, data: bytes) -> None:
        self.written.extend(data)

    def read_available(self, max_bytes: int = 512) -> bytes:
        chunk = bytes(self._to_read[:max_bytes])
        del self._to_read[:max_bytes]
        return chunk

    def feed(self, data: bytes) -> None:
        self._to_read.extend(data)

    def close(self) -> None:
        self.closed = True
