"""Drone configuration: YAML in, validated dataclasses out.

The config mirrors the flight controller's actual Betaflight settings.
If it drifts from the FC, commands are silently wrong -- there is no
error signal from the drone -- so validation here is deliberately loud.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from . import units


class ConfigError(ValueError):
    """Raised when a config is structurally invalid or self-inconsistent."""


@dataclass(frozen=True)
class SerialConfig:
    port: str = "/dev/ttyUSB0"
    baud: int = 115200
    rc_hz: float = 50.0


@dataclass(frozen=True)
class AxisRates:
    """Betaflight rate parameters for one axis."""

    rc_rate: float
    srate: float
    expo: float = 0.0

    @property
    def is_linear(self) -> bool:
        return units.is_linear(self.rc_rate, self.srate)

    @property
    def max_rate(self) -> float:
        """Maximum commandable rate, deg/s."""
        return units.max_rate(self.rc_rate, self.srate, self.expo)


@dataclass(frozen=True)
class BetaflightConfig:
    angle_limit: float = 60.0
    roll: AxisRates = field(default_factory=lambda: AxisRates(67, 67, 0))
    pitch: AxisRates = field(default_factory=lambda: AxisRates(67, 67, 0))
    yaw: AxisRates = field(default_factory=lambda: AxisRates(67, 67, 0))
    rates_type: str = "actual"
    deadband: int = 0
    yaw_deadband: int = 0
    min_check: int = 1050

    def axis(self, name: str) -> AxisRates:
        try:
            return getattr(self, name)
        except AttributeError as exc:
            raise ConfigError(f"unknown axis {name!r}") from exc


@dataclass(frozen=True)
class ModeConfig:
    """One Betaflight aux mode range.

    Betaflight's activation test is half-open (`>= start && < end`), so
    :meth:`value_us` targets the centre of the range rather than an edge.
    """

    aux: int
    range_us: tuple[int, int]

    @property
    def value_us(self) -> float:
        lo, hi = self.range_us
        return (lo + hi) / 2.0


@dataclass(frozen=True)
class ChannelMap:
    roll: int = 0
    pitch: int = 1
    throttle: int = 2
    yaw: int = 3


@dataclass(frozen=True)
class SafetyConfig:
    command_timeout: float = 0.2
    max_angle: float = 30.0
    max_rate: float = 200.0


@dataclass(frozen=True)
class DroneConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)
    channels: ChannelMap = field(default_factory=ChannelMap)
    betaflight: BetaflightConfig = field(default_factory=BetaflightConfig)
    modes: dict[str, ModeConfig] = field(default_factory=dict)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    # ── Loading ───────────────────────────────────────────────────────────────
    @classmethod
    def from_yaml(cls, path: str | Path) -> DroneConfig:
        raw = yaml.safe_load(Path(path).read_text())
        if not isinstance(raw, dict):
            raise ConfigError(f"{path}: expected a YAML mapping at the top level")
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict) -> DroneConfig:
        cfg = cls(
            serial=SerialConfig(**raw.get("serial", {})),
            channels=ChannelMap(**raw.get("channels", {})),
            betaflight=_parse_betaflight(raw.get("betaflight", {})),
            modes=_parse_modes(raw.get("modes", {})),
            safety=SafetyConfig(**raw.get("safety", {})),
        )
        cfg.validate()
        return cfg

    # ── Validation ────────────────────────────────────────────────────────────
    def validate(self) -> None:
        bf = self.betaflight

        if not 10 <= bf.angle_limit <= 80:
            raise ConfigError(
                f"angle_limit {bf.angle_limit} outside Betaflight's range 10..80"
            )

        if bf.rates_type != "actual":
            raise ConfigError(
                f"rates_type {bf.rates_type!r} not supported yet (only 'actual')"
            )

        if self.safety.max_angle > bf.angle_limit:
            raise ConfigError(
                f"safety.max_angle ({self.safety.max_angle}) exceeds "
                f"angle_limit ({bf.angle_limit}) -- the FC would clamp it"
            )

        # Angle mode routes through the rate curve, so a non-linear curve
        # means set_angle() must invert it. That works, but it only stays
        # correct while these numbers match the FC exactly.
        for name in ("roll", "pitch", "yaw"):
            ax = bf.axis(name)
            if not ax.is_linear:
                warnings.warn(
                    f"{name} rates are non-linear (rc_rate={ax.rc_rate}, "
                    f"srate={ax.srate}). Angle commands will be inverted "
                    f"numerically, which is only correct if these match the "
                    f"flight controller. Set rc_rate == srate in Betaflight "
                    f"for an exactly linear response.",
                    stacklevel=3,
                )

        for name, mode in self.modes.items():
            lo, hi = mode.range_us
            if not (900 <= lo < hi <= 2100):
                raise ConfigError(
                    f"mode {name!r}: range {mode.range_us} must satisfy "
                    f"900 <= start < end <= 2100"
                )
            if mode.aux < 1:
                raise ConfigError(f"mode {name!r}: aux must be >= 1")

    def mode(self, name: str) -> ModeConfig:
        try:
            return self.modes[name]
        except KeyError:
            known = ", ".join(sorted(self.modes)) or "(none configured)"
            raise ConfigError(f"unknown mode {name!r}; known modes: {known}") from None


def _parse_betaflight(raw: dict) -> BetaflightConfig:
    rates = raw.get("rates", {})
    rates_type = rates.get("type", "actual")

    def axis(name: str) -> AxisRates:
        return AxisRates(
            rc_rate=_per_axis(rates, "rc_rate", name, 67),
            srate=_per_axis(rates, "srate", name, 67),
            expo=_per_axis(rates, "expo", name, 0),
        )

    known = {"angle_limit", "rates", "deadband", "yaw_deadband", "min_check"}
    if unknown := set(raw) - known:
        raise ConfigError(f"unknown betaflight keys: {sorted(unknown)}")

    return BetaflightConfig(
        angle_limit=raw.get("angle_limit", 60.0),
        roll=axis("roll"),
        pitch=axis("pitch"),
        yaw=axis("yaw"),
        rates_type=rates_type,
        deadband=raw.get("deadband", 0),
        yaw_deadband=raw.get("yaw_deadband", 0),
        min_check=raw.get("min_check", 1050),
    )


def _per_axis(rates: dict, key: str, axis: str, default: float) -> float:
    """Read `rates[key][axis]`, allowing a scalar to apply to all axes."""
    value = rates.get(key, default)
    if isinstance(value, dict):
        return value.get(axis, default)
    return value


def _parse_modes(raw: dict) -> dict[str, ModeConfig]:
    modes = {}
    for name, spec in raw.items():
        try:
            lo, hi = spec["range"]
            modes[name] = ModeConfig(aux=spec["aux"], range_us=(int(lo), int(hi)))
        except (KeyError, TypeError, ValueError) as exc:
            raise ConfigError(
                f"mode {name!r}: expected {{aux: N, range: [lo, hi]}}"
            ) from exc
    return modes
