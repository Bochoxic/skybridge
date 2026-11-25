"""Altitude hold by a trained policy: throttle from a neural network.

The policy is trained in simulation (uav_reinforcement_learning_control,
``train.py --task althold``) and exported there with ``export_policy.py`` to a
numpy bundle, so nothing here needs torch. Like :mod:`.altitude`, it takes
over throttle only: the pilot keeps flying roll, pitch and yaw, and the policy
is told those sticks, because in angle mode they say how far the drone is about
to tilt.

The observation must be exactly the one the policy was trained on, so
:class:`AltHoldObserver` is a copy of the simulator's, and the bundle carries
episodes recorded in the simulator: on load, this observer and MLP replay them
and the bundle is refused if they give anything else.

Camera data is used the way the simulator delivers it: the latest real
detection (not the tracker's coasting prediction), with its age since capture.
When that age passes ``camera_timeout`` the throttle goes to 0 and stays there
until the switch is cycled (``on_timeout: zero``), or the stick is handed back
(``on_timeout: pilot``).
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import numpy as np

from ..config import ConfigError, DroneConfig
from .base import ControlContext, ControlOutput, Disengage, Sticks

FORMAT = "althold-policy/1"
REPO = Path(__file__).resolve().parents[3]  # relative policy paths start here
FRESH = 1e-3  # seconds: since_update below this means the frame itself was a detection


# ── observation: a copy of envs/althold_env.py:AltHoldObserver ──────────────
class AltHoldObserver:
    """Builds the policy's observation. Keep in step with the simulator's copy:
    the parity check on load catches any difference."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.stick_history = int(cfg["stick_history"])
        self.throttle_history = int(cfg["throttle_history"])
        scales = cfg["obs"]
        self.err_scale = float(scales["err"])
        self.vz_scale = float(scales["vz"])
        self.int_limit = float(scales["int"])
        self.age_scale = float(scales["age"])
        self.int_zone = float(cfg["int_zone"])
        self.angle_full = math.radians(float(cfg["angle_full_deg"]))
        self.yaw_full = math.radians(float(cfg["yaw_full_dps"]))
        self.tilt_full = 1.0 / math.cos(self.angle_full) ** 2 - 1.0
        self.thr_lo, self.thr_hi = (float(x) for x in cfg["throttle_range"])
        self.dt = float(cfg["control_dt"])
        self.size = 4 + 4 * self.stick_history + self.throttle_history
        self.target = 0.0
        self.integral = 0.0
        self._started = False
        self._sticks: deque = deque(maxlen=self.stick_history)
        self._throttles: deque = deque(maxlen=self.throttle_history)

    def throttle(self, action: float) -> float:
        a = min(max(float(action), -1.0), 1.0)
        return self.thr_lo + (a + 1.0) / 2.0 * (self.thr_hi - self.thr_lo)

    def action_for(self, throttle: float) -> float:
        a = 2.0 * (throttle - self.thr_lo) / (self.thr_hi - self.thr_lo) - 1.0
        return min(max(float(a), -1.0), 1.0)

    def _stick_features(self, sticks) -> tuple[float, ...]:
        roll, pitch, yaw = (float(x) for x in sticks)
        tilt = 1.0 / max(math.cos(roll) * math.cos(pitch), 0.1) - 1.0
        return (roll / self.angle_full, pitch / self.angle_full, yaw / self.yaw_full,
                tilt / self.tilt_full)

    def engage(self, z: float, throttle: float, sticks) -> None:
        self.target = float(z)
        self.integral = 0.0
        self._started = False
        self._sticks = deque([self._stick_features(sticks)] * self.stick_history,
                             maxlen=self.stick_history)
        self._throttles = deque([self.action_for(throttle)] * self.throttle_history,
                                maxlen=self.throttle_history)

    def observe(self, z: float, vz: float, age: float, sticks) -> np.ndarray:
        err = self.target - float(z)
        if self._started and abs(err) < self.int_zone:
            self.integral = float(np.clip(self.integral + err * self.dt,
                                          -self.int_limit, self.int_limit))
        self._started = True
        self._sticks.appendleft(self._stick_features(sticks))
        head = [err / self.err_scale, float(vz) / self.vz_scale, self.integral / self.int_limit,
                min(max(float(age), 0.0) / self.age_scale, 1.0)]
        obs = np.concatenate([head, np.ravel(list(self._sticks)), list(self._throttles)])
        return np.clip(obs, -1.0, 1.0).astype(np.float32)

    def push_throttle(self, action: float) -> None:
        self._throttles.appendleft(min(max(float(action), -1.0), 1.0))

    def angles(self, sticks: Sticks) -> tuple[float, float, float]:
        """Deflections -> [roll rad, pitch rad, yaw rate rad/s], as angle mode reads them."""
        return (sticks.roll * self.angle_full, sticks.pitch * self.angle_full,
                sticks.yaw * self.yaw_full)


# ── the network ─────────────────────────────────────────────────────────────
class MlpPolicy:
    """The exported actor: dense layers, ReLU or tanh between them, output clipped to ±1."""

    def __init__(self, layers: list[tuple[np.ndarray, np.ndarray]], activation: str) -> None:
        if activation not in ("relu", "tanh"):
            raise ConfigError(f"policy: unsupported activation {activation!r}")
        for i, (w, b) in enumerate(layers):
            if w.ndim != 2 or b.shape != (w.shape[0],):
                raise ConfigError(f"policy: layer {i} has shapes {w.shape}, {b.shape}")
            if i and w.shape[1] != layers[i - 1][0].shape[0]:
                raise ConfigError(f"policy: layer {i} does not fit layer {i - 1}")
        self.layers = [(w.astype(np.float32), b.astype(np.float32)) for w, b in layers]
        self.activation = activation
        self.input_size = self.layers[0][0].shape[1]

    def __call__(self, obs: np.ndarray) -> float:
        x = np.asarray(obs, dtype=np.float32)
        last = len(self.layers) - 1
        for i, (w, b) in enumerate(self.layers):
            x = w @ x + b
            if i < last:
                x = np.maximum(x, 0.0) if self.activation == "relu" else np.tanh(x)
        return float(np.clip(x[0], -1.0, 1.0))


@dataclass(frozen=True)
class PolicyBundle:
    meta: dict[str, Any]
    policy: MlpPolicy
    path: Path

    @property
    def observer_config(self) -> dict[str, Any]:
        return self.meta["observer"]


def load_bundle(path: str | Path) -> PolicyBundle:
    """Read an exported policy and check it against its recorded episodes."""
    path = Path(path).expanduser()
    if not path.is_absolute():
        path = REPO / path
    if not path.exists():
        raise ConfigError(f"policy: {path} not found (export it with export_policy.py)")
    with np.load(path, allow_pickle=False) as data:
        arrays = {k: data[k] for k in data.files}
    try:
        meta = json.loads(str(arrays["meta"]))
    except (KeyError, ValueError) as exc:
        raise ConfigError(f"policy: {path} has no readable meta") from exc
    if meta.get("format") != FORMAT:
        raise ConfigError(f"policy: {path} is {meta.get('format')!r}, expected {FORMAT!r}")
    n = sum(1 for k in arrays if k.startswith("layer") and k.endswith("_w"))
    layers = [(arrays[f"layer{i}_w"], arrays[f"layer{i}_b"]) for i in range(n)]
    policy = MlpPolicy(layers, meta["activation"])
    bundle = PolicyBundle(meta, policy, path)
    if policy.input_size != AltHoldObserver(bundle.observer_config).size:
        raise ConfigError(f"policy: input size {policy.input_size} does not match its observer")
    check_parity(bundle, arrays)
    return bundle


def check_parity(bundle: PolicyBundle, arrays: dict[str, np.ndarray]) -> None:
    """Replay the simulator's recorded observer calls; refuse any difference."""
    try:
        inputs, want_obs, want_act = (arrays[k] for k in
                                      ("parity_inputs", "parity_obs", "parity_actions"))
    except KeyError as exc:
        raise ConfigError(f"policy: {bundle.path} has no parity data") from exc
    tol = bundle.meta.get("parity_tolerance", {"obs": 1e-5, "action": 1e-4})
    observer = AltHoldObserver(bundle.observer_config)
    worst_obs = worst_act = 0.0
    for row, obs_ref, act_ref in zip(inputs, want_obs, want_act, strict=True):
        kind, a, b, c, *sticks = row
        if kind == 0:
            observer.engage(a, b, sticks)
            continue
        obs = observer.observe(a, b, c, sticks)
        action = bundle.policy(obs)
        worst_obs = max(worst_obs, float(np.max(np.abs(obs - obs_ref))))
        worst_act = max(worst_act, abs(action - float(act_ref)))
        observer.push_throttle(float(act_ref))  # as the simulator did
    if worst_obs > tol["obs"] or worst_act > tol["action"]:
        raise ConfigError(
            f"policy: {bundle.path} does not reproduce the simulator "
            f"(observation off by {worst_obs:.1e}, action by {worst_act:.1e}): "
            "the observer or network here differs from the simulator's; re-export the policy "
            "or bring control/policy.py in step with envs/althold_env.py")


# ── the controller ──────────────────────────────────────────────────────────
@dataclass(frozen=True)
class PolicyAltitudeHoldParams:
    policy: str = "models/althold_policy.npz"
    camera_timeout: float = 1.0
    on_timeout: str = "zero"
    min_engage_throttle: float = 0.15
    max_engage_climb: float = 0.5
    max_engage_age: float = 0.2
    max_error: float = 0.6

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> PolicyAltitudeHoldParams:
        known = {f.name for f in fields(cls)}
        if unknown := set(raw) - known:
            raise ConfigError(f"altitude_policy: unknown params {sorted(unknown)}")
        params = cls(**raw)
        if params.on_timeout not in ("zero", "pilot"):
            raise ConfigError("altitude_policy: on_timeout must be 'zero' or 'pilot'")
        for name in ("camera_timeout", "max_engage_climb", "max_engage_age", "max_error"):
            if getattr(params, name) <= 0:
                raise ConfigError(f"altitude_policy: {name} must be > 0")
        if params.min_engage_throttle < 0:
            raise ConfigError("altitude_policy: min_engage_throttle must be >= 0")
        return params


class PolicyAltitudeHold:
    name = "altitude_policy"
    axes = frozenset({"throttle"})

    def __init__(self, params: PolicyAltitudeHoldParams, bundle: PolicyBundle) -> None:
        self.params = params
        self.bundle = bundle
        self.observer = AltHoldObserver(bundle.observer_config)
        self._z = self._vz = 0.0
        self._measured_at = -math.inf     # capture time of the last detection used
        self._timed_out = False

    @classmethod
    def from_params(cls, raw: dict[str, Any]) -> PolicyAltitudeHold:
        params = PolicyAltitudeHoldParams.from_dict(raw)
        return cls(params, load_bundle(params.policy))

    def check_config(self, config: DroneConfig) -> None:
        """The sticks must mean the angles and rates the policy was trained with."""
        obs = self.bundle.observer_config
        bf = config.betaflight
        if not math.isclose(obs["angle_full_deg"], bf.angle_limit, abs_tol=1e-6):
            raise ConfigError(
                f"altitude_policy: trained for angle_limit {obs['angle_full_deg']:g}, "
                f"drone.yaml has {bf.angle_limit:g}: retrain or re-export")
        if not math.isclose(obs["yaw_full_dps"], bf.yaw.max_rate, rel_tol=1e-6):
            raise ConfigError(
                f"altitude_policy: trained for {obs['yaw_full_dps']:g} deg/s full yaw, "
                f"drone.yaml gives {bf.yaw.max_rate:g}")
        rc_period = 1.0 / config.serial.rc_hz
        if not math.isclose(obs["control_dt"], rc_period, rel_tol=1e-3):
            raise ConfigError(
                f"altitude_policy: trained at {1 / obs['control_dt']:g} Hz, RC runs at "
                f"{config.serial.rc_hz:g} Hz")

    def engage(self, ctx: ControlContext) -> None:
        p = self.params
        latest = ctx.estimate
        if latest is None:
            raise Disengage("no track")
        e = latest.value
        if e.status != "detected" or not e.valid or e.pos is None or e.vel is None:
            raise Disengage(f"track is {e.status}")
        if ctx.t - e.t > p.max_engage_age:
            raise Disengage(f"estimate stale ({ctx.t - e.t:.2f} s)")
        if ctx.sticks.throttle < p.min_engage_throttle:
            raise Disengage("throttle too low")
        if abs(e.vel[2]) > p.max_engage_climb:
            raise Disengage(f"moving too fast ({e.vel[2]:+.1f} m/s)")
        self._z, self._vz, self._measured_at = e.pos[2], e.vel[2], e.t
        self._timed_out = False
        self.observer.engage(self._z, ctx.sticks.throttle, self.observer.angles(ctx.sticks))

    def update(self, ctx: ControlContext) -> ControlOutput:
        return self._step(ctx, sent=None)

    def shadow_update(self, ctx: ControlContext, sent: float) -> ControlOutput:
        """Run without flying: the throttle actually sent (another controller's, or the
        pilot's) goes into the history, so the output is what this policy would send
        if it took over now."""
        return self._step(ctx, sent=sent)

    def _debug(self, **values: float) -> dict[str, float]:
        keys = ("setpoint", "z", "vz", "error", "integral", "age", "action", "throttle", "sent",
                "timed_out", "shadow")
        return {k: float(values.get(k, math.nan)) for k in keys}

    def _step(self, ctx: ControlContext, sent: float | None) -> ControlOutput:
        p = self.params
        self._take(ctx)
        age = ctx.t - self._measured_at
        if age > p.camera_timeout and not self._timed_out:
            if p.on_timeout == "pilot":
                raise Disengage(f"no altitude for {age:.1f} s")
            self._timed_out = True
        if self._timed_out:
            if sent is not None:
                self.observer.push_throttle(self.observer.action_for(sent))
            return ControlOutput(throttle=0.0, text=f"ALT-RL NO CAMERA {age:.1f}s T0%",
                                 debug=self._debug(age=age, throttle=0.0, timed_out=1.0,
                                                   sent=0.0 if sent is None else sent,
                                                   shadow=float(sent is not None)))

        error = self.observer.target - self._z
        if abs(error) > p.max_error:
            raise Disengage(f"height error {error:+.2f} m")
        sticks = self.observer.angles(ctx.sticks)
        obs = self.observer.observe(self._z, self._vz, age, sticks)
        action = self.bundle.policy(obs)
        throttle = self.observer.throttle(action)
        self.observer.push_throttle(action if sent is None else self.observer.action_for(sent))
        return ControlOutput(
            throttle=throttle,
            text=f"ALT-RL {self._z:.2f}>{self.observer.target:.2f}m T{throttle * 100:.0f}%",
            debug=self._debug(setpoint=self.observer.target, z=self._z, vz=self._vz, error=error,
                              integral=self.observer.integral, age=age, action=action,
                              throttle=throttle, sent=throttle if sent is None else sent,
                              timed_out=0.0, shadow=float(sent is not None)),
        )

    def _take(self, ctx: ControlContext) -> None:
        """Keep the newest real detection. Coasting predictions are not measurements:
        the policy was trained to hold the last one and watch its age grow."""
        latest = ctx.estimate
        if latest is None:
            return
        e = latest.value
        if (not e.valid or e.pos is None or e.vel is None or e.since_update is None
                or e.since_update > FRESH or e.t <= self._measured_at):
            return
        self._z, self._vz, self._measured_at = e.pos[2], e.vel[2], e.t
