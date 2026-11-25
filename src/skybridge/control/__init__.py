"""Controllers the pilot can switch on during RC flight. See :mod:`.base`."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from ..bus import Bus
from ..config import ConfigError, DroneConfig
from ..state import TelemetryState, Timestamped
from .altitude import AltitudeHold, AltitudeHoldParams
from .base import ControlContext, Controller, ControlOutput, Disengage, Sticks
from .policy import PolicyAltitudeHold, PolicyAltitudeHoldParams
from .supervisor import ControlStatus, ControlSupervisor

# Name in configs/drone.yaml -> factory taking that entry's params dict.
CONTROLLERS: dict[str, Callable[[dict[str, Any]], Controller]] = {
    AltitudeHold.name: AltitudeHold.from_params,
    PolicyAltitudeHold.name: PolicyAltitudeHold.from_params,
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
        controller = factory(entry.params)
        # Optional: a controller that depends on the drone's settings checks them here.
        if check := getattr(controller, "check_config", None):
            check(config)
        controllers[name] = controller
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
    "PolicyAltitudeHold",
    "PolicyAltitudeHoldParams",
    "Sticks",
    "build_supervisor",
]
