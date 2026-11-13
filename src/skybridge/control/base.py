"""The interface between the RC loop and a flight controller.

A controller is a small class with a name, the axes it drives, and two methods.
It sees the pilot's sticks and the latest perception and telemetry, and returns
values only for the axes it wants. Everything common to all controllers (the
enabling switch, arm and flight-mode checks, stick override, output limits,
surviving a crash) lives in :class:`skybridge.control.ControlSupervisor`.

To add a controller: write the class, add it to ``CONTROLLERS`` in
``skybridge/control/__init__.py``, and give it an entry under
``control.controllers`` in ``configs/drone.yaml``.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Protocol

from ..state import TelemetryState, Timestamped

if TYPE_CHECKING:
    from ..perception.types import DroneEstimate


class Disengage(Exception):
    """Raised by a controller to refuse or give up. The message reaches the pilot."""


@dataclass(frozen=True)
class Sticks:
    """The pilot's command as sent: deflection -1..1, throttle 0..1."""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0


@dataclass(frozen=True)
class ControlContext:
    """Everything a controller may look at for one RC tick."""

    t: float                                        # monotonic seconds
    dt: float                                       # seconds since the previous tick
    sticks: Sticks
    estimate: Timestamped[DroneEstimate] | None     # age is measured from capture
    telemetry: TelemetryState


@dataclass(frozen=True)
class ControlOutput:
    """One tick of output. An axis left as None stays on the pilot's stick."""

    roll: float | None = None       # deflection, -1..1
    pitch: float | None = None      # deflection, -1..1
    yaw: float | None = None        # deflection, -1..1
    throttle: float | None = None   # 0..1
    text: str = ""                  # short, for the status line
    debug: Mapping[str, float] = field(default_factory=dict)  # plotted when logging


class Controller(Protocol):
    name: str
    axes: frozenset[str]  # what it drives; must be allowed by its config entry

    def engage(self, ctx: ControlContext) -> None:
        """Start from the current state. Raise :class:`Disengage` to refuse."""
        ...

    def update(self, ctx: ControlContext) -> ControlOutput:
        """One tick. Raise :class:`Disengage` to give control back."""
        ...
