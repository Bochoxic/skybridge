"""Joystick axes -> CRSF channel ticks.

EdgeTX exports its mixer outputs as joystick axes, -100%..+100% mapped
onto -32767..32767. Over its own ELRS link the radio sends the same
-100%..+100% as CRSF ticks 172..1811 (988..2012us as Betaflight decodes
them -- confirmed on the Boxer, 2026-11-07). Mapping axis full scale onto
tick full scale therefore reproduces exactly what the radio would have
transmitted itself.
"""

from __future__ import annotations

from collections.abc import Sequence

from ..config import DroneConfig, RcConfig
from ..crsf.constants import CH_MAX, CH_MID, CH_MIN, NUM_CHANNELS
from ..units import clamp_tick, tick_to_us

AXIS_MAX = 32767


def axis_to_tick(value: int) -> int:
    """Map a joystick axis value onto a CRSF tick: -32767/0/32767 -> 172/992/1811.

    Piecewise so both endpoints land exactly despite the tick range not
    being symmetric about 992 (820 ticks below, 819 above).
    """
    span = (CH_MAX - CH_MID) if value >= 0 else (CH_MID - CH_MIN)
    return clamp_tick(round(CH_MID + value / AXIS_MAX * span))


class RcMapping:
    """Builds a full 16-channel tick array from an :class:`RcFrame`'s axes.

    Unmapped channels take the same neutral values the Bridge uses: sticks
    centred, throttle and aux channels low.
    """

    def __init__(self, rc: RcConfig, neutral: Sequence[int]) -> None:
        if len(neutral) != NUM_CHANNELS:
            raise ValueError(f"neutral must have {NUM_CHANNELS} channels")
        self._entries = dict(rc.mapping)
        self._neutral = tuple(neutral)

    @classmethod
    def from_config(cls, cfg: DroneConfig) -> RcMapping:
        cmap = cfg.channels
        neutral = [CH_MIN] * NUM_CHANNELS
        for idx in (cmap.roll, cmap.pitch, cmap.yaw):
            neutral[idx] = CH_MID
        return cls(cfg.rc, neutral)

    @property
    def max_axis(self) -> int:
        """Highest axis index the mapping needs, or -1 if none."""
        return max((e.axis for e in self._entries.values()), default=-1)

    def apply(self, axes: Sequence[int]) -> list[int]:
        ticks = list(self._neutral)
        for channel, entry in self._entries.items():
            if entry.axis >= len(axes):
                raise ValueError(
                    f"CH{channel + 1} is mapped to axis {entry.axis}, but the "
                    f"device only has {len(axes)} axes"
                )
            value = axes[entry.axis]
            ticks[channel] = axis_to_tick(-value if entry.invert else value)
        return ticks


def active_modes(cfg: DroneConfig, ticks: Sequence[int]) -> list[str]:
    """Names of the configured modes whose aux range the ticks fall in."""
    return [
        name
        for name, mode in cfg.modes.items()
        if mode.is_active(tick_to_us(ticks[mode.channel]))
    ]
