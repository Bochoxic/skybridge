"""The Bridge: threaded CRSF RC control with telemetry.

The bridge owns a background thread that transmits RC frames at a steady
rate no matter what the caller's control loop is doing. Control code
calls the ``set_*`` methods whenever it likes and reads :attr:`state`;
the RC stream keeps flowing regardless.

One thread does both the write and the read. At 115200 baud a 50Hz tick
carries a couple of hundred bytes, so draining and decoding costs tens of
microseconds against a 20ms budget. Two threads sharing one pyserial
object would be a genuine correctness hazard, and guarding it with a lock
is worse -- a blocking read holds the lock and starves the RC writer.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, replace

from . import units
from .config import DroneConfig
from .crsf.constants import CH_MIN, NUM_CHANNELS
from .crsf.decoders import DECODERS
from .crsf.protocol import parse_frames, rc_frame
from .state import (
    Altitude,
    Attitude,
    Battery,
    LinkStats,
    TelemetryState,
    Timestamped,
)
from .transport import SerialTransport, Transport


class BridgeError(RuntimeError):
    """Base class for bridge failures."""


class ModeMismatch(BridgeError):
    """A command was issued that the active flight mode does not accept.

    The same stick value means ten degrees in Angle mode and hundreds of
    degrees per second in Acro, so silently reinterpreting a command
    would be the most dangerous bug this library could have.
    """


class ArmRefused(BridgeError):
    """Arming was refused because preconditions were not met."""


@dataclass(frozen=True)
class Command:
    """A complete set of commanded inputs.

    Frozen and replaced wholesale, so the RC thread can never observe a
    new roll paired with a stale pitch.
    """

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0
    armed: bool = False
    mode: str | None = None
    aux_us: tuple[tuple[int, float], ...] = ()


class Bridge:
    """Send RC over CRSF and read CRSF telemetry back.

    Usage::

        with Bridge(cfg) as bridge:
            bridge.set_mode("angle")
            bridge.arm()
            bridge.set_angle(roll=10)
    """

    def __init__(
        self,
        config: DroneConfig,
        transport: Transport | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
        auto_start: bool = False,
    ) -> None:
        self.config = config
        self._clock = clock
        self._transport = transport or SerialTransport(
            config.serial.port, config.serial.baud
        )

        self._lock = threading.Lock()
        self._command = Command()
        self._last_command_at = clock()
        self._command_seen = False

        self._state = TelemetryState()
        self._buf = bytearray()
        self._att_count = 0
        self._last_rate_at = clock()

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._failsafe_active = False

        if auto_start:
            self.start()

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="skybridge-rc", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Disarm, stop the RC thread and close the port."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        # Leave the link in a safe state rather than latching the last
        # command, which is what the original reference script did.
        try:
            self._transport.write(self._build_frame(Command()))
        except Exception:
            pass
        self._transport.close()

    close = stop

    def __enter__(self) -> Bridge:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    # ── Commands ──────────────────────────────────────────────────────────────
    def _update(self, **changes: object) -> None:
        with self._lock:
            self._command = replace(self._command, **changes)
            self._last_command_at = self._clock()
            self._command_seen = True
            self._failsafe_active = False

    def set_angle(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> None:
        """Command attitude in degrees (and yaw in deg/s).

        Requires the active mode to be an angle-style mode.
        """
        self._require_mode_accepts("angle")

        bf = self.config.betaflight
        limit = self.config.safety.max_angle

        roll_d = units.deflection_for_angle(
            _clamp(roll, limit), bf.angle_limit,
            bf.roll.rc_rate, bf.roll.srate, bf.roll.expo,
        )
        pitch_d = units.deflection_for_angle(
            _clamp(pitch, limit), bf.angle_limit,
            bf.pitch.rc_rate, bf.pitch.srate, bf.pitch.expo,
        )
        yaw_d = units.deflection_for_rate(
            _clamp(yaw_rate, self.config.safety.max_rate),
            bf.yaw.rc_rate, bf.yaw.srate, bf.yaw.expo,
        )
        self._update(roll=roll_d, pitch=pitch_d, yaw=yaw_d)

    def set_rates(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        """Command angular rates in deg/s (acro)."""
        self._require_mode_accepts("acro")

        bf = self.config.betaflight
        limit = self.config.safety.max_rate

        self._update(
            roll=units.deflection_for_rate(
                _clamp(roll, limit), bf.roll.rc_rate, bf.roll.srate, bf.roll.expo
            ),
            pitch=units.deflection_for_rate(
                _clamp(pitch, limit), bf.pitch.rc_rate, bf.pitch.srate, bf.pitch.expo
            ),
            yaw=units.deflection_for_rate(
                _clamp(yaw, limit), bf.yaw.rc_rate, bf.yaw.srate, bf.yaw.expo
            ),
        )

    def set_throttle(self, throttle: float) -> None:
        """Set throttle, 0.0 to 1.0."""
        self._update(throttle=max(0.0, min(1.0, throttle)))

    def set_mode(self, name: str) -> None:
        """Select a named flight mode from the config."""
        self.config.mode(name)  # raises if unknown
        self._update(mode=name)

    def set_aux(self, aux: int, value_us: float) -> None:
        """Set an aux channel directly, for anything not in the config."""
        with self._lock:
            others = tuple(
                (a, v) for a, v in self._command.aux_us if a != aux
            )
            self._command = replace(
                self._command, aux_us=others + ((aux, value_us),)
            )
            self._last_command_at = self._clock()
            self._command_seen = True
            self._failsafe_active = False

    def arm(self, *, force: bool = False) -> None:
        """Arm the drone.

        Refuses unless throttle is at minimum, which is the classic way
        to hurt someone. Pass ``force=True`` only if you mean it.
        """
        if "arm" not in self.config.modes:
            raise ArmRefused("no 'arm' mode configured")

        with self._lock:
            throttle = self._command.throttle
        if throttle > 0.0 and not force:
            raise ArmRefused(
                f"refusing to arm with throttle at {throttle:.2f}; "
                f"call set_throttle(0) first"
            )
        self._update(armed=True)

    def disarm(self) -> None:
        """Disarm. Always permitted."""
        self._update(armed=False, throttle=0.0)

    # ── Telemetry ─────────────────────────────────────────────────────────────
    @property
    def state(self) -> TelemetryState:
        """A coherent snapshot of the latest telemetry."""
        with self._lock:
            snapshot = self._state
        return snapshot.at(self._clock())

    @property
    def failsafe_active(self) -> bool:
        """True if the watchdog has tripped and is holding a safe output."""
        with self._lock:
            return self._failsafe_active

    # ── Thread body ───────────────────────────────────────────────────────────
    def _run(self) -> None:
        period = 1.0 / self.config.serial.rc_hz
        next_tick = self._clock()

        while not self._stop.is_set():
            self.tick()

            # Absolute deadline, not sleep(period): sleeping a fixed
            # period *after* doing the work makes the real rate drift
            # below the target.
            next_tick += period
            delay = next_tick - self._clock()
            if delay < 0:
                next_tick = self._clock()
            else:
                self._stop.wait(delay)

    def tick(self) -> None:
        """One RC cycle: watchdog, transmit, receive. Public for testing."""
        now = self._clock()

        with self._lock:
            command = self._command
            stale = (
                self._command_seen
                and (now - self._last_command_at)
                > self.config.safety.command_timeout
            )
            if stale:
                # Immediate disarm: cut ARM and throttle outright.
                command = Command(mode=command.mode)
                self._command = command
                self._failsafe_active = True

        self._transport.write(self._build_frame(command))
        self._read_telemetry(now)

    def _build_frame(self, command: Command) -> bytes:
        ch = [units.deflection_to_tick(0.0)] * NUM_CHANNELS
        cmap = self.config.channels

        ch[cmap.roll] = units.deflection_to_tick(command.roll)
        ch[cmap.pitch] = units.deflection_to_tick(command.pitch)
        ch[cmap.yaw] = units.deflection_to_tick(command.yaw)

        # Throttle spans the full range from its low endpoint.
        ch[cmap.throttle] = units.us_to_tick(
            units.US_MIN + command.throttle * (units.US_MAX - units.US_MIN)
        )

        # Aux channels default low so an unconfigured switch reads "off".
        for idx in range(NUM_CHANNELS):
            if idx not in (cmap.roll, cmap.pitch, cmap.throttle, cmap.yaw):
                ch[idx] = CH_MIN

        if command.mode:
            mode = self.config.mode(command.mode)
            ch[_aux_index(mode.aux)] = units.us_to_tick(mode.value_us)

        if command.armed and "arm" in self.config.modes:
            arm = self.config.modes["arm"]
            ch[_aux_index(arm.aux)] = units.us_to_tick(arm.value_us)

        for aux, value_us in command.aux_us:
            ch[_aux_index(aux)] = units.us_to_tick(value_us)

        return rc_frame(ch)

    def _read_telemetry(self, now: float) -> None:
        data = self._transport.read_available()
        if not data:
            self._maybe_update_rate(now)
            return

        self._buf.extend(data)
        updates: dict[str, object] = {}

        for ftype, payload in parse_frames(self._buf):
            entry = DECODERS.get(ftype)
            if entry is None:
                continue
            name, decode = entry
            values = decode(payload)
            if not values:
                continue

            if name == "ATTITUDE":
                self._att_count += 1
                updates["attitude"] = Timestamped(
                    Attitude(
                        roll_deg=values["roll_deg"],
                        pitch_deg=values["pitch_deg"],
                        yaw_deg=values["yaw_deg"],
                    ),
                    now,
                )
            elif name == "BATTERY":
                updates["battery"] = Timestamped(
                    Battery(
                        voltage_V=values["voltage_V"],
                        current_A=values["current_A"],
                        capacity_mAh=values["capacity_mAh"],
                        remaining_pct=values["remaining_%"],
                    ),
                    now,
                )
            elif name == "LINK_STATS":
                updates["link"] = Timestamped(
                    LinkStats(
                        uplink_rssi_dbm=values["uplink_rssi1"],
                        uplink_lq=values["uplink_lq"],
                        uplink_snr=values["uplink_snr"],
                        downlink_lq=values["downlink_lq"],
                        rf_mode=values["rf_mode"],
                    ),
                    now,
                )
            elif name == "BARO_ALT":
                updates["altitude"] = Timestamped(
                    Altitude(
                        alt_m=values["alt_m"],
                        vario_ms=values.get("vario_ms", 0.0),
                    ),
                    now,
                )
            elif name == "FLIGHT_MODE":
                updates["flight_mode"] = Timestamped(values["mode"], now)

        if updates:
            with self._lock:
                self._state = replace(self._state, **updates)

        self._maybe_update_rate(now)

    def _maybe_update_rate(self, now: float) -> None:
        elapsed = now - self._last_rate_at
        if elapsed >= 1.0:
            hz = self._att_count / elapsed
            self._att_count = 0
            self._last_rate_at = now
            with self._lock:
                self._state = replace(self._state, attitude_hz=hz)

    # ── Internals ─────────────────────────────────────────────────────────────
    def _require_mode_accepts(self, kind: str) -> None:
        with self._lock:
            active = self._command.mode

        if active is None or active not in self.config.modes:
            return  # nothing selected yet; caller is explicit elsewhere

        if kind == "angle" and active == "acro":
            raise ModeMismatch(
                "set_angle() called while mode is 'acro': the same stick "
                "value means degrees in angle mode and deg/s in acro. "
                "Call set_mode('angle') or use set_rates()."
            )
        if kind == "acro" and active == "angle":
            raise ModeMismatch(
                "set_rates() called while mode is 'angle'. "
                "Call set_mode('acro') or use set_angle()."
            )


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _aux_index(aux: int) -> int:
    """AUX1 is channel 5 (index 4) in the standard AETR layout."""
    return 3 + aux
