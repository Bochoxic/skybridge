"""The trained-policy altitude hold: bundle loading, parity, and its flight rules."""

import json
import math

import numpy as np
import pytest
from test_control import ALT_ENTRY, BASE, HIGH, HOVER, LOW, MID, THR, Rig, control

from skybridge import ConfigError, DroneConfig
from skybridge.control import build_supervisor
from skybridge.control.policy import (
    FORMAT,
    AltHoldObserver,
    MlpPolicy,
    PolicyAltitudeHold,
    PolicyAltitudeHoldParams,
    load_bundle,
)
from skybridge.units import throttle_fraction, throttle_to_tick

OBSERVER = {
    "stick_history": 5, "throttle_history": 10, "angle_full_deg": 60.0, "yaw_full_dps": 670.0,
    "throttle_range": [0.05, 0.56], "obs": {"err": 1.0, "vz": 2.0, "int": 0.5, "age": 0.5},
    "int_zone": 0.3, "control_dt": 0.02,
}
HOLD = 0.33  # the throttle the constant test policy asks for


def make_bundle(path, *, constant=True, observer=OBSERVER, seed=0):
    """A small bundle whose parity episodes come from this module's own observer.
    `constant`: the output layer is zero, so the policy always asks for HOLD."""
    rng = np.random.default_rng(seed)
    obs = AltHoldObserver(observer)
    w0 = rng.normal(0, 0.3, (16, obs.size)).astype(np.float32)
    b0 = rng.normal(0, 0.1, 16).astype(np.float32)
    w1 = np.zeros((1, 16), np.float32) if constant else rng.normal(0, 0.3, (1, 16)).astype(np.float32)
    b1 = np.array([obs.action_for(HOLD)], np.float32)
    policy = MlpPolicy([(w0, b0), (w1, b1)], "relu")

    inputs, observations, actions = [], [], []
    obs.engage(1.0, 0.3, (0.0, 0.0, 0.0))
    inputs.append([0, 1.0, 0.3, 0, 0, 0, 0])
    observations.append(np.full(obs.size, np.nan))
    actions.append(np.nan)
    for k in range(50):
        row = [1, 1.0 + 0.1 * math.sin(k / 5), 0.2 * math.cos(k / 5), 0.03 * (k % 3),
               0.2 * math.sin(k / 7), -0.1, 0.5]
        o = obs.observe(*row[1:4], row[4:])
        a = policy(o)
        obs.push_throttle(a)
        inputs.append(row)
        observations.append(o)
        actions.append(a)
    meta = {"format": FORMAT, "run": "test", "activation": "relu", "observer": observer}
    np.savez(path, meta=np.array(json.dumps(meta)), layer0_w=w0, layer0_b=b0, layer1_w=w1,
             layer1_b=b1, parity_inputs=np.array(inputs), parity_obs=np.array(observations),
             parity_actions=np.array(actions))
    return path


def policy_rig(tmp_path, **params):
    bundle = load_bundle(make_bundle(tmp_path / "policy.npz"))
    controller = PolicyAltitudeHold(PolicyAltitudeHoldParams(**params), bundle)
    return Rig(controllers={"altitude_policy": controller},
               entries={"altitude_policy": ALT_ENTRY})


def sent_throttle(status):
    return throttle_fraction(status.ticks[THR])


# ── The bundle ────────────────────────────────────────────────────────────────
def test_bundle_loads_and_reproduces_its_episodes(tmp_path):
    bundle = load_bundle(make_bundle(tmp_path / "p.npz", constant=False))
    assert bundle.policy.input_size == AltHoldObserver(OBSERVER).size


def test_changed_weights_are_refused(tmp_path):
    path = make_bundle(tmp_path / "p.npz", constant=False)
    data = dict(np.load(path))
    data["layer1_b"] = data["layer1_b"] + 0.01
    np.savez(path, **data)
    with pytest.raises(ConfigError, match="does not reproduce"):
        load_bundle(path)


def test_wrong_format_is_refused(tmp_path):
    path = make_bundle(tmp_path / "p.npz")
    data = dict(np.load(path))
    data["meta"] = np.array(json.dumps({"format": "something-else"}))
    np.savez(path, **data)
    with pytest.raises(ConfigError, match="expected"):
        load_bundle(path)


def test_missing_bundle_says_how_to_make_one(tmp_path):
    with pytest.raises(ConfigError, match="export_policy"):
        load_bundle(tmp_path / "nope.npz")


def test_stick_scaling_must_match_the_drone(tmp_path):
    controller = PolicyAltitudeHold(PolicyAltitudeHoldParams(),
                                    load_bundle(make_bundle(tmp_path / "p.npz")))
    controller.check_config(DroneConfig.from_dict(BASE))   # angle_limit 60, yaw 670 deg/s
    other = DroneConfig.from_dict({**BASE, "betaflight": {"angle_limit": 20},
                                   "safety": {**BASE["safety"], "max_angle": 15}})
    with pytest.raises(ConfigError, match="angle_limit"):
        controller.check_config(other)


def test_build_supervisor_checks_the_drone(tmp_path):
    path = make_bundle(tmp_path / "p.npz")
    entry = {**ALT_ENTRY, "params": {"policy": str(path)}}
    cfg = DroneConfig.from_dict({**BASE, "betaflight": {"angle_limit": 20},
                                 "safety": {**BASE["safety"], "max_angle": 15},
                                 "control": {"controllers": {"altitude_policy": entry}}})
    with pytest.raises(ConfigError, match="angle_limit"):
        build_supervisor(cfg, lambda: None)


@pytest.mark.parametrize("params", [{"on_timeout": "hover"}, {"camera_timeout": 0}, {"bogus": 1}])
def test_bad_params_are_rejected(params):
    with pytest.raises(ConfigError):
        PolicyAltitudeHoldParams.from_dict(params)


# ── Flying ────────────────────────────────────────────────────────────────────
def test_engages_and_sends_the_policy_throttle(tmp_path):
    rig = policy_rig(tmp_path)
    status = rig.engage()
    assert control(status).engaged
    assert sent_throttle(status) == pytest.approx(HOLD, abs=0.002)
    assert "ALT-RL" in control(status).text


def test_only_throttle_changes(tmp_path):
    rig = policy_rig(tmp_path)
    rig.radio.set(roll=HIGH)
    rig.step()
    status = rig.engage()
    pilot = rig.pt.mapping.apply(rig.source.latest().axes)
    assert [t for i, t in enumerate(status.ticks) if i != THR] == \
        [t for i, t in enumerate(pilot) if i != THR]


def test_coasting_is_held_not_used(tmp_path):
    """Predictions while coasting are not measurements: the last detection stays in use
    and its age grows, as in training."""
    rig = policy_rig(tmp_path)
    rig.engage()
    hold = rig.supervisor._slots[0].controller
    rig.since_update, rig.z = 0.2, 0.7     # coasting, and the prediction has wandered
    rig.step()
    assert hold._z == pytest.approx(1.0)
    assert control(rig.step()).debug["age"] > 0.05


def test_camera_timeout_sends_zero_and_latches(tmp_path):
    rig = policy_rig(tmp_path, camera_timeout=0.5)
    rig.engage()
    rig.no_estimate = True
    for _ in range(20):                     # 0.4 s: still flying on the last detection
        assert sent_throttle(rig.step()) == pytest.approx(HOLD, abs=0.002)
    for _ in range(10):
        status = rig.step()
    assert control(status).engaged
    assert status.ticks[THR] == throttle_to_tick(0.0)
    assert "NO CAMERA" in control(status).text

    rig.no_estimate = False                 # the camera is back: still zero
    for _ in range(5):
        assert rig.step().ticks[THR] == throttle_to_tick(0.0)

    rig.radio.set(switch=LOW)               # cycling the switch gives the stick back
    assert rig.step().ticks[THR] == rig.stick_tick()


def test_camera_timeout_can_hand_back_instead(tmp_path):
    rig = policy_rig(tmp_path, camera_timeout=0.5, on_timeout="pilot")
    rig.engage()
    rig.no_estimate = True
    for _ in range(30):
        status = rig.step()
    assert not control(status).engaged
    assert "no altitude" in control(status).note
    assert status.ticks[THR] == rig.stick_tick()


def test_large_height_error_hands_back(tmp_path):
    rig = policy_rig(tmp_path, max_error=0.4)
    rig.engage()
    rig.z = 1.5
    status = rig.step()
    assert "height error" in control(status).note
    assert status.ticks[THR] == rig.stick_tick()


def test_throttle_stick_hands_back(tmp_path):
    rig = policy_rig(tmp_path)
    rig.engage()
    rig.radio.set(throttle=HOVER + 12000)
    assert "throttle stick moved" in control(rig.step()).note


@pytest.mark.parametrize("setup, note", [
    (lambda r: setattr(r, "no_estimate", True), "no track"),
    (lambda r: setattr(r, "status", "coasting"), "track is coasting"),
    (lambda r: setattr(r, "age", 0.5), "stale"),
    (lambda r: setattr(r, "vz", 1.0), "moving too fast"),
    (lambda r: r.radio.set(throttle=-30000), "throttle too low"),
])
def test_refuses_to_engage(tmp_path, setup, note):
    rig = policy_rig(tmp_path)
    setup(rig)
    rig.step()
    status = rig.engage()
    assert not control(status).engaged
    assert note in control(status).note


def test_disarm_disengages(tmp_path):
    rig = policy_rig(tmp_path)
    rig.engage()
    rig.radio.set(arm=LOW)
    assert not control(rig.step()).engaged


# ── Shadow mode: running alongside the PID, never flying ─────────────────────
PID_ENTRY = {"switch": {"aux": 3, "range": [1300, 1700]}, "axes": ["throttle"],
             "require_mode": "angle"}
POLICY_ENTRY = {"switch": {"aux": 3, "range": [1700, 2100]}, "axes": ["throttle"],
                "require_mode": "angle", "shadow": "altitude_hold"}


def pid_and_policy_rig(tmp_path, shadow=True):
    from skybridge.control import AltitudeHold, AltitudeHoldParams
    bundle = load_bundle(make_bundle(tmp_path / "policy.npz"))
    entry = POLICY_ENTRY if shadow else {k: v for k, v in POLICY_ENTRY.items() if k != "shadow"}
    return Rig(controllers={"altitude_hold": AltitudeHold(AltitudeHoldParams()),
                            "altitude_policy": PolicyAltitudeHold(PolicyAltitudeHoldParams(), bundle)},
               entries={"altitude_hold": PID_ENTRY, "altitude_policy": entry})


def by_name(status, name):
    return next(c for c in status.control if c.name == name)


def test_shadow_runs_while_the_pid_flies_and_sends_nothing(tmp_path):
    with_shadow, without = pid_and_policy_rig(tmp_path), pid_and_policy_rig(tmp_path, shadow=False)
    for rig in (with_shadow, without):
        rig.radio.set(switch=MID)
    sent = []
    for k in range(30):
        for rig in (with_shadow, without):
            rig.z = 1.0 - 0.005 * k         # sinking: the PID pushes throttle up
        a, b = with_shadow.step(), without.step()
        assert list(a.ticks) == list(b.ticks)       # the shadow changes nothing sent
        sent.append(throttle_fraction(a.ticks[THR]))
    pid, policy = by_name(a, "altitude_hold"), by_name(a, "altitude_policy")
    assert pid.engaged and not policy.engaged
    assert policy.shadow and policy.text.startswith("shadow ALT-RL")
    assert policy.debug["shadow"] == 1.0
    assert policy.debug["sent"] == pytest.approx(sent[-1], abs=0.002)   # it saw the PID's throttle
    assert policy.debug["throttle"] == pytest.approx(HOLD, abs=0.002)  # and says what it would send


def test_switching_to_the_policy_after_shadow_engages_it_for_real(tmp_path):
    rig = pid_and_policy_rig(tmp_path)
    rig.radio.set(switch=MID)
    for _ in range(10):
        rig.step()
    rig.radio.set(switch=HIGH)
    status = rig.step()
    policy = by_name(status, "altitude_policy")
    assert policy.engaged and not policy.shadow
    assert not by_name(status, "altitude_hold").engaged
    assert sent_throttle(status) == pytest.approx(HOLD, abs=0.002)


def test_shadow_stops_with_the_pid(tmp_path):
    rig = pid_and_policy_rig(tmp_path)
    rig.radio.set(switch=MID)
    rig.step()
    rig.radio.set(switch=LOW)
    policy = by_name(rig.step(), "altitude_policy")
    assert not policy.shadow and policy.text == ""


def test_a_failing_shadow_does_not_disturb_the_pid(tmp_path):
    rig = pid_and_policy_rig(tmp_path)
    rig.supervisor._slots[1].controller.update = None     # would crash if called directly
    rig.supervisor._slots[1].controller.shadow_update = lambda ctx, sent: 1 / 0
    rig.radio.set(switch=MID)
    status = rig.step()
    assert by_name(status, "altitude_hold").engaged
    assert "shadow off" in by_name(status, "altitude_policy").text


def test_shadow_must_name_another_controller():
    entry = {**POLICY_ENTRY, "shadow": "nope"}
    with pytest.raises(ConfigError, match="shadow"):
        DroneConfig.from_dict({**BASE, "control": {"controllers": {
            "altitude_hold": PID_ENTRY, "altitude_policy": entry}}})
