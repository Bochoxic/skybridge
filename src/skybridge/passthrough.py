"""RC passthrough: fly from a physical radio, through the laptop.

    RcSource ──axes──► RcMapping ──ticks──► guards ──ticks──► Bridge.set_channels

Channels are forwarded raw, so the drone flies exactly as it would on the
radio's own link: Betaflight applies rates, and the switches select arm
and modes. An optional :class:`skybridge.control.ControlSupervisor` may then
replace individual stick channels. The laptop otherwise adds only guards:

1. **Startup arm interlock.** The arm channel is held disarmed until the
   arm switch has been seen *off* at least once. Plugging in (or starting
   the script) with the switch already up must never arm the drone.
2. **Throttle-high arm refusal.** Turning arm on while throttle is above
   ``betaflight.min_check`` is held disarmed until the switch is cycled --
   the same intent as :meth:`Bridge.arm`'s guard. Betaflight refuses this
   too; refusing it here as well keeps one layer from depending on the other.
3. **Throttle cap** (optional). The throttle channel is scaled so full
   stick gives ``max_throttle``, keeping the whole stick travel usable.
4. **Dead source.** If the radio disappears (USB unplugged), the bridge
   simply stops being fed and its ``command_timeout`` watchdog disarms.
   There is deliberately no second failsafe path to get wrong.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .bridge import Bridge
from .config import DroneConfig
from .crsf.constants import CH_MAX, CH_MIN
from .rc.mapping import RcMapping
from .rc.source import RcSource
from .units import throttle_fraction, tick_to_us  # noqa: F401  (throttle_fraction re-exported)

if TYPE_CHECKING:
    from .control.supervisor import ControlStatus, ControlSupervisor

# Guard notes, shown to the pilot.
ARM_OK = ""
ARM_WAIT_OFF = "arm switch was on at start: switch it OFF first"
ARM_THROTTLE = "throttle was up when arming: throttle down, cycle the arm switch"


class ArmGuard:
    """Guards 1 and 2 above. Stateful: call :meth:`apply` once per tick."""

    def __init__(self, config: DroneConfig) -> None:
        arm = config.modes.get("arm")
        self._arm = arm
        self._throttle_ch = config.channels.throttle
        self._min_check_us = config.betaflight.min_check
        if arm is not None:
            # A value Betaflight will read as "not armed", whatever the range.
            self._off_tick = CH_MIN if not arm.is_active(tick_to_us(CH_MIN)) else CH_MAX
        self._seen_off = False
        self._blocked = False
        self._was_armed = False
        self.note = ARM_OK

    @property
    def armed(self) -> bool:
        """Whether the last output had the arm channel active."""
        return self._was_armed

    def apply(self, ticks: list[int]) -> list[int]:
        if self._arm is None:
            return ticks
        ch = self._arm.channel
        if not self._arm.is_active(tick_to_us(ticks[ch])):
            self._seen_off = True
            self._blocked = False
            self._was_armed = False
            self.note = ARM_OK
            return ticks

        if not self._seen_off:
            self.note = ARM_WAIT_OFF
        elif self._blocked:
            self.note = ARM_THROTTLE
        elif not self._was_armed and tick_to_us(ticks[self._throttle_ch]) >= self._min_check_us:
            self._blocked = True
            self.note = ARM_THROTTLE
        else:
            self._was_armed = True
            self.note = ARM_OK
            return ticks

        ticks = list(ticks)
        ticks[ch] = self._off_tick
        self._was_armed = False
        return ticks


def cap_throttle(ticks: Sequence[int], channel: int, max_throttle: float) -> list[int]:
    """Scale the throttle channel so full stick gives `max_throttle` of the range."""
    out = list(ticks)
    if max_throttle < 1.0:
        out[channel] = round(CH_MIN + (out[channel] - CH_MIN) * max_throttle)
    return out


@dataclass(frozen=True)
class PassthroughStatus:
    state: str                      # "waiting" | "live" | "dead"
    ticks: tuple[int, ...] = ()     # what was sent this step (empty unless live)
    armed: bool = False
    note: str = ""
    control: tuple[ControlStatus, ...] = ()  # one per configured controller


class RcPassthrough:
    """Feeds a :class:`Bridge` from an :class:`RcSource` with guards applied."""

    def __init__(
        self,
        bridge: Bridge,
        source: RcSource,
        config: DroneConfig,
        *,
        max_throttle: float | None = None,
        supervisor: ControlSupervisor | None = None,
    ) -> None:
        self.bridge = bridge
        self.source = source
        self.config = config
        self.mapping = RcMapping.from_config(config)
        self.max_throttle = config.rc.max_throttle if max_throttle is None else max_throttle
        if not 0.0 < self.max_throttle <= 1.0:
            raise ValueError(f"max_throttle {self.max_throttle} must be in (0, 1]")
        self.guard = ArmGuard(config)
        self.supervisor = supervisor
        self._throttle_ceiling = cap_throttle([CH_MAX], 0, self.max_throttle)[0]

    def step(self) -> PassthroughStatus:
        """Read the radio, apply guards, send. One call per RC tick."""
        if not self.source.alive:
            # Stop feeding: the bridge watchdog disarms after command_timeout.
            return PassthroughStatus("dead", note="radio disconnected")

        frame = self.source.latest()
        if frame is None:
            return PassthroughStatus("waiting", note="waiting for radio state")

        ticks = self.mapping.apply(frame.axes)
        ticks = cap_throttle(ticks, self.config.channels.throttle, self.max_throttle)
        ticks = self.guard.apply(ticks)
        control: tuple[ControlStatus, ...] = ()
        if self.supervisor is not None:
            # Last, so the cap cannot rescale a controller's output and the
            # arm guard keeps judging the pilot's real stick.
            ticks = self.supervisor.apply(
                ticks, armed=self.guard.armed, throttle_ceiling=self._throttle_ceiling
            )
            control = self.supervisor.status
        self.bridge.set_channels(ticks)
        return PassthroughStatus("live", tuple(ticks), self.guard.armed, self.guard.note, control)

    def run(
        self,
        stop: threading.Event,
        on_status: Callable[[PassthroughStatus], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Call :meth:`step` at the RC rate until `stop` is set."""
        period = 1.0 / self.config.serial.rc_hz
        next_tick = clock()
        while not stop.is_set():
            status = self.step()
            if on_status is not None:
                on_status(status)
            next_tick += period
            delay = next_tick - clock()
            if delay < 0:
                next_tick = clock()
            else:
                stop.wait(delay)


