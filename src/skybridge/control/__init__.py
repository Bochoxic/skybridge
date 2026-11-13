"""Controllers the pilot can switch on during RC flight. See :mod:`.base`."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from ..bus import Bus
from ..config import ConfigError, DroneConfig
from ..state import TelemetryState, Timestamped
from .altitude import AltitudeHold, AltitudeHoldParams
from .base import ControlContext, Controller, ControlOutput, Disengage, Sticks
from .supervisor import ControlStatus, ControlSupervisor

# Name in configs/drone.yaml -> factory taking that entry's params dict.
CONTROLLERS: dict[str, Callable[[dict[str, Any]], Controller]] = {
    AltitudeHold.name: AltitudeHold.from_params,
}


def build_supervisor(
    config: DroneConfig,
    estimates: Callable[[], Timestamped | None],
    telemetry: Callable[[], TelemetryState] = TelemetryState,
    *,
    bus: Bus | None = None,
) -> ControlSupervisor:
    """Build every controller under ``control.controllers``. Fails loudly."""
    entries = config.control.controllers
    if not entries:
        raise ConfigError("control.controllers is empty: nothing to switch on")
    controllers = {}
    for name, entry in entries.items():
        factory = CONTROLLERS.get(name)
        if factory is None:
            known = ", ".join(sorted(CONTROLLERS))
            raise ConfigError(f"unknown controller {name!r}; known: {known}")
        controllers[name] = factory(entry.params)
    return ControlSupervisor(config, controllers, estimates, telemetry, bus=bus)


__all__ = [
    "CONTROLLERS",
    "AltitudeHold",
    "AltitudeHoldParams",
    "ControlContext",
    "ControlOutput",
    "ControlStatus",
    "ControlSupervisor",
    "Controller",
    "Disengage",
    "Sticks",
    "build_supervisor",
]
