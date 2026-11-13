"""Decides which controllers are running and applies what they return.

Runs once per RC tick, on the RC loop's thread, after the arm guard. Each
configured controller has its own switch range, allowed axes and state. The
rules are the same for all of them:

* A controller engages only at the moment its switch **enters** its range. So
  after a refusal or a drop-out it stays off until the switch leaves the range
  and comes back. It never re-engages by itself: a throttle change the pilot
  did not ask for is the failure this rule exists to prevent.
* Any drop-out returns the pilot's sticks in the same tick.
* Nothing a controller does can stop the RC stream.
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field

from ..bus import Bus
from ..config import AXES, ConfigError, ControllerConfig, DroneConfig
from ..crsf.constants import CH_MAX
from ..state import TelemetryState, Timestamped
from ..units import (
    deflection_to_tick,
    throttle_fraction,
    throttle_to_tick,
    tick_to_deflection,
    tick_to_us,
)
from .base import ControlContext, Controller, ControlOutput, Disengage, Sticks

log = logging.getLogger(__name__)

TOPIC = "control/status"
FAULT = "controller fault"


@dataclass(frozen=True)
class ControlStatus:
    name: str
    engaged: bool = False
    note: str = ""      # why it isn't running, when the pilot needs to know
    text: str = ""      # the controller's own status while running
    debug: Mapping[str, float] = field(default_factory=dict)


@dataclass
class _Slot:
    controller: Controller
    config: ControllerConfig
    engaged: bool = False
    in_range: bool = False
    note: str = ""
    text: str = ""
    debug: Mapping[str, float] = field(default_factory=dict)
    engage_sticks: Sticks = field(default_factory=Sticks)

    def status(self) -> ControlStatus:
        return ControlStatus(self.controller.name, self.engaged, self.note, self.text, self.debug)


class ControlSupervisor:
    def __init__(
        self,
        config: DroneConfig,
        controllers: Mapping[str, Controller],
        estimates: Callable[[], Timestamped | None],
        telemetry: Callable[[], TelemetryState] = TelemetryState,
        *,
        bus: Bus | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._config = config
        self._estimates = estimates
        self._telemetry = telemetry
        self._bus = bus
        self._clock = clock
        self._last_t: float | None = None
        self._published: dict[str, ControlStatus] = {}
        self._slots: list[_Slot] = []
        for name, controller in controllers.items():
            ctl = config.control.controllers.get(name)
            if ctl is None:
                raise ConfigError(f"controller {name!r} has no entry under control.controllers")
            if extra := controller.axes - ctl.axes:
                raise ConfigError(
                    f"controller {name!r} drives {sorted(extra)}, "
                    f"but its config only allows {sorted(ctl.axes)}"
                )
            self._slots.append(_Slot(controller, ctl))
        self.status: tuple[ControlStatus, ...] = tuple(s.status() for s in self._slots)

    def apply(
        self,
        ticks: Sequence[int],
        *,
        armed: bool,
        throttle_ceiling: int = CH_MAX,
    ) -> list[int]:
        """Return `ticks` with the running controllers' axes replaced."""
        pilot = list(ticks)
        try:
            out = self._apply(pilot, armed, throttle_ceiling)
        except Exception:
            log.exception("control supervisor failed; returning the sticks")
            for slot in self._slots:
                if slot.engaged:
                    self._drop(slot, FAULT)
            out = pilot
        self.status = tuple(s.status() for s in self._slots)
        self._publish()
        return out

    def _apply(self, pilot: list[int], armed: bool, ceiling: int) -> list[int]:
        now = self._clock()
        dt = 0.0 if self._last_t is None else now - self._last_t
        self._last_t = now
        ctx = ControlContext(now, dt, self._sticks(pilot), self._estimates(), self._telemetry())

        entered = []
        for slot in self._slots:
            in_range = slot.config.switch.is_active(tick_to_us(pilot[slot.config.switch.channel]))
            if in_range and not slot.in_range:
                entered.append(slot)
            slot.in_range = in_range
            if not in_range:
                slot.engaged = False
                slot.note = slot.text = ""
                slot.debug = {}

        # Stop first, so a 3-position switch hands over within this tick.
        for slot in self._slots:
            if not slot.engaged:
                continue
            if reason := self._must_stop(slot, pilot, ctx.sticks, armed, ceiling):
                self._drop(slot, reason)
        for slot in entered:
            self._try_engage(slot, pilot, ctx, armed)

        out = list(pilot)
        for slot in self._slots:
            if slot.engaged:
                self._run(slot, ctx, out, ceiling)
        return out

    def _sticks(self, ticks: Sequence[int]) -> Sticks:
        cm = self._config.channels
        return Sticks(
            roll=tick_to_deflection(ticks[cm.roll]),
            pitch=tick_to_deflection(ticks[cm.pitch]),
            yaw=tick_to_deflection(ticks[cm.yaw]),
            throttle=throttle_fraction(ticks[cm.throttle]),
        )

    def _mode_active(self, slot: _Slot, ticks: Sequence[int]) -> bool:
        mode = self._config.modes[slot.config.require_mode]
        return mode.is_active(tick_to_us(ticks[mode.channel]))

    def _must_stop(
        self, slot: _Slot, ticks: Sequence[int], sticks: Sticks, armed: bool, ceiling: int
    ) -> str:
        """Why this engaged controller has to stop, or "" if it may continue."""
        if not armed:
            return "disarmed"
        if not self._mode_active(slot, ticks):
            return f"left {slot.config.require_mode} mode"
        for axis in slot.controller.axes:
            limit = slot.config.stick_override
            if axis == "throttle":
                # Sticks are after the --max-throttle scaling; the override is
                # meant in physical stick travel.
                limit *= throttle_fraction(ceiling)
            if abs(getattr(sticks, axis) - getattr(slot.engage_sticks, axis)) > limit:
                return f"{axis} stick moved"
        return ""

    def _try_engage(
        self, slot: _Slot, ticks: Sequence[int], ctx: ControlContext, armed: bool
    ) -> None:
        reason = ""
        if not armed:
            reason = "not armed"
        elif not self._mode_active(slot, ticks):
            reason = f"needs {slot.config.require_mode} mode"
        else:
            try:
                slot.controller.engage(ctx)
            except Disengage as exc:
                reason = str(exc)
            except Exception:
                log.exception("controller %s failed to engage", slot.controller.name)
                reason = FAULT
        if reason:
            self._drop(slot, reason)
            return
        slot.engaged = True
        slot.note = ""
        slot.engage_sticks = ctx.sticks

    def _run(self, slot: _Slot, ctx: ControlContext, ticks: list[int], ceiling: int) -> None:
        try:
            output = slot.controller.update(ctx)
        except Disengage as exc:
            self._drop(slot, str(exc))
            return
        except Exception:
            log.exception("controller %s failed", slot.controller.name)
            self._drop(slot, FAULT)
            return

        # Check everything before writing anything: all of the output or none.
        if reason := self._reject(slot, output):
            log.error("controller %s: %s", slot.controller.name, reason)
            self._drop(slot, reason)
            return

        cm = self._config.channels
        if output.throttle is not None:
            ticks[cm.throttle] = min(throttle_to_tick(output.throttle), ceiling)
        for axis, channel in (("roll", cm.roll), ("pitch", cm.pitch), ("yaw", cm.yaw)):
            value = getattr(output, axis)
            if value is not None:
                ticks[channel] = deflection_to_tick(value)
        slot.text = output.text
        slot.debug = output.debug

    @staticmethod
    def _reject(slot: _Slot, output: ControlOutput) -> str:
        for axis in AXES:
            value = getattr(output, axis)
            if value is None:
                continue
            if axis not in slot.controller.axes:
                return f"output on undeclared axis {axis}"
            if not math.isfinite(value):
                return f"non-finite {axis} output"
        return ""

    @staticmethod
    def _drop(slot: _Slot, reason: str) -> None:
        slot.engaged = False
        slot.note = f"{reason}, cycle switch"
        slot.text = ""
        slot.debug = {}

    def _publish(self) -> None:
        if self._bus is None or self._last_t is None:
            return
        for status in self.status:
            previous = self._published.get(status.name)
            # Every tick while running, for the plots; otherwise only on a change.
            if status.engaged or status != previous:
                self._published[status.name] = status
                self._bus.publish(TOPIC, status, self._last_t)
