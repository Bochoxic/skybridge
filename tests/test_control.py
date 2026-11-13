"""Controllers: the supervisor's rules, altitude hold, and a closed-loop check."""

import math
import time

import pytest

from skybridge import Bridge, Bus, ConfigError, DroneConfig, LoopbackTransport, RcPassthrough
from skybridge.control import (
    AltitudeHold,
    AltitudeHoldParams,
    ControlContext,
    ControlOutput,
    ControlStatus,
    ControlSupervisor,
    Disengage,
    Sticks,
    build_supervisor,
)
from skybridge.control.supervisor import TOPIC
from skybridge.crsf.constants import CH_MAX
from skybridge.perception.link import PerceptionLink
from skybridge.perception.types import DroneEstimate
from skybridge.perception.viz import RerunBusSink
from skybridge.rc import FakeRcSource
from skybridge.state import TelemetryState, Timestamped
from skybridge.units import throttle_fraction, throttle_to_tick

LOW, MID, HIGH = -32767, 0, 32767
THR, ROLL = 2, 0
HOVER = -13107  # axis value for 30% of the stick range

BASE = {
    "serial": {"port": "fake", "rc_hz": 50},
    "modes": {
        "arm": {"aux": 1, "range": [1700, 2100]},
        "angle": {"aux": 2, "range": [900, 1300]},
        "acro": {"aux": 2, "range": [1400, 2000]},
    },
    "safety": {"command_timeout": 0.2},
}
ALT_ENTRY = {
    "switch": {"aux": 3, "range": [1700, 2100]},
    "axes": ["throttle"],
    "require_mode": "angle",
}


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class Radio:
    """Sticks and switches, as the axes the passthrough reads."""

    def __init__(self, source):
        self.source = source
        self.state = {"roll": MID, "pitch": MID, "throttle": HOVER, "yaw": MID,
                      "arm": HIGH, "flight": LOW, "switch": LOW}

    def set(self, **changes):
        self.state.update(changes)
        s = self.state
        self.source.set((s["roll"], s["pitch"], s["throttle"], s["yaw"],
                         s["arm"], s["flight"], s["switch"]))


class Rig:
    def __init__(self, controllers=None, entries=None, max_throttle=None):
        entries = entries if entries is not None else {"altitude_hold": ALT_ENTRY}
        self.cfg = DroneConfig.from_dict({**BASE, "control": {"controllers": entries}})
        self.clock = FakeClock()
        self.source = FakeRcSource()
        self.radio = Radio(self.source)
        self.transport = LoopbackTransport()
        self.bridge = Bridge(self.cfg, self.transport, clock=self.clock)
        self.bus = Bus()
        self.z = 1.0
        self.vz = 0.0
        self.status = "detected"
        self.valid = True
        self.since_update = 0.0
        self.age = 0.03
        self.no_estimate = False
        controllers = controllers or {"altitude_hold": AltitudeHold(AltitudeHoldParams())}
        self.supervisor = ControlSupervisor(self.cfg, controllers, self.estimate, bus=self.bus,
                                            clock=self.clock)
        self.pt = RcPassthrough(self.bridge, self.source, self.cfg, max_throttle=max_throttle,
                                supervisor=self.supervisor)
        # Pass through once with arm off, so the arm guard has seen the switch low.
        self.radio.set(arm=LOW, throttle=LOW)
        self.step()
        self.radio.set(arm=HIGH)
        self.step()
        self.radio.set(throttle=HOVER)
        self.step()

    def estimate(self):
        if self.no_estimate:
            return None
        t = self.clock.now - self.age
        e = DroneEstimate(t=t, seq=1, status=self.status, valid=self.valid,
                          pos=(0.0, 0.0, self.z) if self.valid else None,
                          vel=(0.0, 0.0, self.vz) if self.valid else None,
                          pos_std=(0.01, 0.01, 0.01) if self.valid else None,
                          since_update=self.since_update if self.valid else None)
        return Timestamped(e, t).at(self.clock.now)

    def step(self, dt=0.02):
        self.clock.now += dt
        return self.pt.step()

    def engage(self):
        self.radio.set(switch=HIGH)
        return self.step()

    def stick_tick(self):
        """The throttle tick the pilot's stick alone would give."""
        ticks = self.pt.mapping.apply(self.source.latest().axes)
        return ticks[self.cfg.channels.throttle]


def control(status):
    return status.control[0]


# ── Engaging ──────────────────────────────────────────────────────────────────
def test_engages_without_a_throttle_bump():
    rig = Rig()
    before = rig.step().ticks[THR]
    status = rig.engage()
    assert control(status).engaged
    assert status.ticks[THR] == before


def test_only_throttle_changes_while_engaged():
    rig = Rig()
    rig.radio.set(roll=HIGH)
    rig.step()
    rig.engage()
    rig.z = 0.8  # below the target: the controller must add throttle
    status = rig.step()
    pilot = rig.pt.mapping.apply(rig.source.latest().axes)
    assert status.ticks[THR] > pilot[THR]
    assert [t for i, t in enumerate(status.ticks) if i != THR] == \
        [t for i, t in enumerate(pilot) if i != THR]


@pytest.mark.parametrize("setup, note", [
    (lambda r: setattr(r, "no_estimate", True), "no track"),
    (lambda r: setattr(r, "valid", False), "track"),
    (lambda r: setattr(r, "status", "coasting"), "track is coasting"),
    (lambda r: setattr(r, "age", 0.5), "stale"),
    (lambda r: setattr(r, "since_update", 0.5), "coasting"),
    (lambda r: setattr(r, "vz", 1.0), "moving too fast"),
    (lambda r: r.radio.set(throttle=-30000), "throttle too low"),
    (lambda r: r.radio.set(flight=HIGH), "needs angle mode"),
])
def test_refuses_to_engage(setup, note):
    rig = Rig()
    setup(rig)
    rig.step()
    status = rig.engage()
    assert not control(status).engaged
    assert note in control(status).note


def test_refused_when_not_armed():
    rig = Rig()
    rig.radio.set(arm=LOW)
    rig.step()
    assert not control(rig.engage()).engaged


def test_switch_on_at_start_does_not_engage_later():
    rig = Rig()
    rig.radio.set(arm=LOW, switch=HIGH, throttle=LOW)
    rig.step()
    rig.radio.set(arm=HIGH)
    rig.step()
    rig.radio.set(throttle=HOVER)
    assert not control(rig.step()).engaged


# ── Dropping out ──────────────────────────────────────────────────────────────
@pytest.mark.parametrize("trigger, note", [
    (lambda r: setattr(r, "no_estimate", True), "no track"),
    (lambda r: setattr(r, "valid", False), "track"),
    (lambda r: setattr(r, "age", 0.5), "stale"),
    (lambda r: setattr(r, "since_update", 0.5), "coasting"),
    (lambda r: setattr(r, "z", 2.0), "height error"),
    (lambda r: r.radio.set(flight=HIGH), "left angle mode"),
    (lambda r: r.radio.set(throttle=HOVER + 12000), "throttle stick moved"),
])
def test_drop_out_returns_the_stick_in_the_same_tick(trigger, note):
    rig = Rig()
    rig.engage()
    rig.z = 0.9  # so the controller's output differs from the stick
    assert rig.step().ticks[THR] != rig.stick_tick()

    trigger(rig)
    status = rig.step()
    assert not control(status).engaged
    assert note in control(status).note
    assert status.ticks[THR] == rig.stick_tick()


def test_coasting_briefly_keeps_holding():
    rig = Rig()
    rig.engage()
    rig.status = "coasting"
    rig.since_update = 0.1
    assert control(rig.step()).engaged


def test_small_stick_move_is_ignored():
    rig = Rig()
    rig.engage()
    rig.radio.set(throttle=HOVER + 3000)
    assert control(rig.step()).engaged


def test_disarm_disengages():
    rig = Rig()
    rig.engage()
    rig.radio.set(arm=LOW)
    status = rig.step()
    assert not control(status).engaged
    assert not status.armed


def test_switch_off_is_silent():
    rig = Rig()
    rig.engage()
    rig.radio.set(switch=LOW)
    status = rig.step()
    assert not control(status).engaged
    assert control(status).note == ""


def test_no_re_engage_until_switch_is_cycled():
    rig = Rig()
    rig.engage()
    rig.since_update = 0.5
    assert not control(rig.step()).engaged

    rig.since_update = 0.0  # the track came back: must not take over by itself
    for _ in range(5):
        assert not control(rig.step()).engaged

    rig.radio.set(switch=LOW)
    rig.step()
    assert control(rig.engage()).engaged


# ── Limits ────────────────────────────────────────────────────────────────────
def test_output_stays_within_authority():
    rig = Rig()
    rig.engage()
    trim = throttle_fraction(rig.step().ticks[THR])
    rig.z = 0.55  # a large error
    for _ in range(100):
        fraction = throttle_fraction(rig.step().ticks[THR])
        assert fraction <= trim + 0.1 + 0.002


def test_max_throttle_is_a_ceiling():
    rig = Rig(max_throttle=0.5)
    rig.radio.set(throttle=HIGH)
    rig.step()
    ceiling = rig.step().ticks[THR]
    rig.engage()
    rig.z = 0.8
    for _ in range(50):
        assert rig.step().ticks[THR] <= ceiling


def test_stick_override_is_in_physical_stick_travel():
    rig = Rig(max_throttle=0.5)
    rig.radio.set(throttle=MID)
    rig.step()
    rig.engage()
    # 0.20 of the stick range: over the 0.15 limit, though it sends only 0.10.
    rig.radio.set(throttle=int(0.4 * HIGH))
    assert "stick moved" in control(rig.step()).note


def test_no_supervisor_changes_nothing():
    cfg = DroneConfig.from_dict(BASE)
    source, transport = FakeRcSource(), LoopbackTransport()
    pt = RcPassthrough(Bridge(cfg, transport, clock=FakeClock()), source, cfg)
    source.set((MID, MID, HOVER, MID, LOW, LOW, HIGH))
    status = pt.step()
    assert status.control == ()
    assert list(status.ticks) == pt.mapping.apply(source.latest().axes)


# ── Controllers that misbehave ────────────────────────────────────────────────
class Fake:
    """A scriptable controller."""

    def __init__(self, name="fake", axes=("throttle",), output=None, refuse=None):
        self.name = name
        self.axes = frozenset(axes)
        self.output = output or ControlOutput(throttle=0.4)
        self.refuse = refuse
        self.raises = None
        self.engaged_at = None

    def engage(self, ctx):
        if self.refuse:
            raise self.refuse
        self.engaged_at = ctx.t

    def update(self, ctx):
        if self.raises:
            raise self.raises
        return self.output


def fake_rig(controller, **entry):
    return Rig({"fake": controller}, {"fake": {**ALT_ENTRY, **entry}})


@pytest.mark.parametrize("output, note", [
    (ControlOutput(throttle=math.nan), "non-finite"),
    (ControlOutput(throttle=math.inf), "non-finite"),
    (ControlOutput(throttle=0.4, roll=0.5), "undeclared axis roll"),
])
def test_bad_output_drops_out_and_rc_continues(output, note):
    controller = Fake()
    rig = fake_rig(controller)
    assert control(rig.engage()).engaged

    controller.output = output
    status = rig.step()
    assert status.state == "live"
    assert note in control(status).note
    assert status.ticks[THR] == rig.stick_tick()


def test_crashing_update_drops_out_and_rc_continues():
    controller = Fake()
    rig = fake_rig(controller)
    rig.engage()
    controller.raises = ZeroDivisionError()
    status = rig.step()
    assert status.state == "live"
    assert "fault" in control(status).note
    assert status.ticks[THR] == rig.stick_tick()


def test_crashing_engage_is_a_refusal():
    rig = fake_rig(Fake(refuse=RuntimeError("boom")))
    status = rig.engage()
    assert status.state == "live" and not control(status).engaged
    assert "fault" in control(status).note


def test_crashing_estimate_source_keeps_rc_alive():
    rig = Rig()
    rig.engage()
    rig.supervisor._estimates = lambda: 1 / 0
    status = rig.step()
    assert status.state == "live"
    assert not control(status).engaged
    assert status.ticks[THR] == rig.stick_tick()


def test_controller_axis_outside_its_config_fails_at_startup():
    cfg = DroneConfig.from_dict({**BASE, "control": {"controllers": {"fake": ALT_ENTRY}}})
    with pytest.raises(ConfigError, match="roll"):
        ControlSupervisor(cfg, {"fake": Fake(axes=("throttle", "roll"))}, lambda: None)


# ── Several controllers ───────────────────────────────────────────────────────
MID_RANGE = {"aux": 3, "range": [1400, 1600]}
LOW_RANGE = {"aux": 3, "range": [900, 1100]}


def test_each_controller_engages_only_in_its_own_range():
    high, middle = Fake("high"), Fake("middle", output=ControlOutput(throttle=0.5))
    rig = Rig({"high": high, "middle": middle},
              {"high": ALT_ENTRY, "middle": {**ALT_ENTRY, "switch": MID_RANGE}})
    status = rig.engage()
    assert [c.engaged for c in status.control] == [True, False]
    assert middle.engaged_at is None


def test_three_position_switch_hands_over_in_one_tick():
    high, middle = Fake("high"), Fake("middle", output=ControlOutput(throttle=0.5))
    rig = Rig({"high": high, "middle": middle},
              {"high": ALT_ENTRY, "middle": {**ALT_ENTRY, "switch": MID_RANGE}})
    rig.engage()
    rig.radio.set(switch=MID)
    status = rig.step()
    assert [c.engaged for c in status.control] == [False, True]
    assert throttle_fraction(status.ticks[THR]) == pytest.approx(0.5, abs=0.002)


def test_controllers_on_different_switches_stay_on_their_own_axes():
    a = Fake("a", output=ControlOutput(throttle=0.4))
    b = Fake("b", axes=("roll",), output=ControlOutput(roll=0.5))
    cfg = {**BASE,
           "rc": {"mapping": {f"ch{i + 1}": {"axis": i} for i in range(8)}},
           "control": {"controllers": {
               "a": ALT_ENTRY,
               "b": {"switch": {"aux": 4, "range": [1700, 2100]}, "axes": ["roll"],
                     "require_mode": "angle"},
           }}}
    config = DroneConfig.from_dict(cfg)
    sup = ControlSupervisor(config, {"a": a, "b": b}, lambda: None, clock=FakeClock())
    ticks = [992] * 16
    ticks[4], ticks[5], ticks[6], ticks[7] = 1811, 172, 1811, 1811  # arm, angle, both switches
    ticks[THR] = throttle_to_tick(0.3)

    out = sup.apply(ticks, armed=True)
    assert [c.engaged for c in sup.status] == [True, True]
    assert throttle_fraction(out[THR]) == pytest.approx(0.4, abs=0.002)
    assert out[ROLL] > 992
    assert [t for i, t in enumerate(out) if i not in (THR, ROLL)] == \
        [t for i, t in enumerate(ticks) if i not in (THR, ROLL)]

    # One failing leaves the other running.
    a.raises = RuntimeError("boom")
    out = sup.apply(ticks, armed=True)
    assert [c.engaged for c in sup.status] == [False, True]
    assert out[THR] == ticks[THR]
    assert out[ROLL] > 992


# ── Building and publishing ───────────────────────────────────────────────────
def test_build_rejects_unknown_controller():
    cfg = DroneConfig.from_dict({**BASE, "control": {"controllers": {"nope": ALT_ENTRY}}})
    with pytest.raises(ConfigError, match="unknown controller"):
        build_supervisor(cfg, lambda: None)


def test_build_rejects_empty_config():
    with pytest.raises(ConfigError, match="empty"):
        build_supervisor(DroneConfig.from_dict(BASE), lambda: None)


def test_build_rejects_unknown_params():
    entry = {**ALT_ENTRY, "params": {"kpp": 1.0}}
    cfg = DroneConfig.from_dict({**BASE, "control": {"controllers": {"altitude_hold": entry}}})
    with pytest.raises(ConfigError, match="kpp"):
        build_supervisor(cfg, lambda: None)


@pytest.mark.parametrize("params", [{"authority": 0.0}, {"authority": 0.9}, {"kp": -1.0},
                                    {"max_age": 0.0}, {"i_limit": 0.5}])
def test_bad_params_are_rejected(params):
    with pytest.raises(ConfigError):
        AltitudeHoldParams.from_dict(params)


def test_status_is_published_each_tick_while_engaged_and_once_when_off():
    rig = Rig()
    seen = []
    rig.bus.subscribe("control/status", lambda topic, payload, t: seen.append(payload))
    rig.step()
    assert seen == []  # nothing changed
    rig.engage()
    rig.step()
    assert len(seen) == 2 and "z" in seen[-1].debug
    rig.radio.set(switch=LOW)
    rig.step()
    rig.step()
    assert len(seen) == 3 and not seen[-1].engaged


def test_flies_on_estimates_received_over_the_real_link():
    """The same wire format and age handling that rc_fly.py uses, on the real clock."""
    link = PerceptionLink("127.0.0.1", 0)
    try:
        cfg = DroneConfig.from_dict({**BASE, "control": {"controllers": {
            "altitude_hold": ALT_ENTRY}}})
        supervisor = build_supervisor(cfg, link.latest)

        def send():
            link.receive(DroneEstimate(
                t=time.monotonic(), seq=1, status="detected", valid=True, pos=(0.0, 0.0, 1.0),
                vel=(0.0, 0.0, 0.0), pos_std=(0.01, 0.01, 0.01), since_update=0.0).to_wire())

        ticks = [992] * 16
        ticks[4], ticks[5], ticks[6] = 1811, 172, 1811  # armed, angle, switch high
        ticks[THR] = throttle_to_tick(0.3)
        send()
        assert supervisor.apply(ticks, armed=True) == ticks
        assert supervisor.status[0].engaged
        send()
        supervisor.apply(ticks, armed=True)
        assert supervisor.status[0].debug["setpoint"] == pytest.approx(1.0)
    finally:
        link.close()


def test_status_can_be_logged_to_rerun(tmp_path):
    import rerun as rr

    rec = rr.RecordingStream("test_control", recording_id="t")
    rec.save(str(tmp_path / "control.rrd"))
    sink = RerunBusSink(Bus(), rec, {})
    # Called directly: on the sink's thread a failure would only be logged.
    sink._log_message(TOPIC, ControlStatus("a", True, text="x", debug={"z": 1.0, "e": 0.1}), 1.0)
    sink._log_message(TOPIC, ControlStatus("a", False, note="track lost, cycle switch"), 1.1)
    rec.flush()
    rec.disconnect()
    assert (tmp_path / "control.rrd").stat().st_size > 0


def test_real_config_loads_and_builds():
    from pathlib import Path

    cfg = DroneConfig.from_yaml(Path(__file__).parent.parent / "configs" / "drone.yaml")
    supervisor = build_supervisor(cfg, lambda: None)
    assert [c.name for c in supervisor.status] == ["altitude_hold"]


# ── Units ─────────────────────────────────────────────────────────────────────
def test_throttle_tick_round_trip_is_exact():
    for tick in range(throttle_to_tick(0.0), throttle_to_tick(1.0) + 1):
        assert throttle_to_tick(throttle_fraction(tick)) == tick


def test_throttle_to_tick_clamps():
    assert throttle_to_tick(-1.0) == throttle_to_tick(0.0)
    assert throttle_to_tick(2.0) == throttle_to_tick(1.0) < CH_MAX


# ── The control law ───────────────────────────────────────────────────────────
def context(t, dt, z, vz=0.0, throttle=0.3, since_update=0.0):
    e = DroneEstimate(t=t, seq=1, status="detected", valid=True, pos=(0.0, 0.0, z),
                      vel=(0.0, 0.0, vz), pos_std=(0.01, 0.01, 0.01), since_update=since_update)
    return ControlContext(t, dt, Sticks(throttle=throttle), Timestamped(e, t).at(t),
                          TelemetryState())


def test_pid_terms_have_the_right_sign():
    hold = AltitudeHold(AltitudeHoldParams(ki=0.0))
    hold.engage(context(0.0, 0.0, 1.0))
    assert hold.update(context(0.02, 0.02, 0.9)).throttle > 0.3     # below target
    assert hold.update(context(0.04, 0.02, 1.1)).throttle < 0.3     # above target
    assert hold.update(context(0.06, 0.02, 1.0, vz=0.5)).throttle < 0.3  # climbing


def test_integrator_stops_at_the_limit():
    hold = AltitudeHold(AltitudeHoldParams(kp=0.0, kd=0.0, ki=1.0, i_limit=0.05, authority=0.1))
    hold.engage(context(0.0, 0.0, 1.0))
    for i in range(500):
        out = hold.update(context(i * 0.02, 0.02, 0.9))
    assert out.throttle == pytest.approx(0.35)


def test_integrator_does_not_wind_up_while_saturated():
    hold = AltitudeHold(AltitudeHoldParams(kp=1.0, ki=0.5, kd=0.0, i_limit=0.08, authority=0.1))
    hold.engage(context(0.0, 0.0, 1.0))
    for i in range(200):
        out = hold.update(context(i * 0.02, 0.02, 0.8))
        assert out.throttle == pytest.approx(0.4)
    assert out.debug["integral"] == 0.0


def test_long_gap_does_not_dump_into_the_integrator():
    hold = AltitudeHold(AltitudeHoldParams(kp=0.0, kd=0.0, ki=0.1))
    hold.engage(context(0.0, 0.0, 1.0))
    out = hold.update(context(5.0, 5.0, 0.9))
    assert out.debug["integral"] == pytest.approx(0.1 * 0.1 * 0.1)


def test_height_is_projected_to_now():
    hold = AltitudeHold()
    hold.engage(context(0.0, 0.0, 1.0))
    e = DroneEstimate(t=1.0, seq=1, status="detected", valid=True, pos=(0.0, 0.0, 1.0),
                      vel=(0.0, 0.0, 0.5), pos_std=(0.0, 0.0, 0.0), since_update=0.0)
    ctx = ControlContext(1.1, 0.02, Sticks(throttle=0.3), Timestamped(e, 1.0).at(1.1),
                         TelemetryState())
    assert hold.update(ctx).debug["z"] == pytest.approx(1.05)


def test_disengage_is_an_exception_the_supervisor_knows():
    hold = AltitudeHold()
    with pytest.raises(Disengage):
        hold.update(ControlContext(0.0, 0.02, Sticks(), None, TelemetryState()))


# ── Closed loop ───────────────────────────────────────────────────────────────
def simulate(gain, hover, trim, seconds=30.0, delay=0.06, rc_hz=50.0, start_vz=0.0):
    """Vertical dynamics with drag, sensor latency and a motor lag.

    `gain` is m/s^2 per unit of throttle. Returns the height over time and the
    final target.
    """
    hold = AltitudeHold()
    dt = 1.0 / rc_hz
    steps = int(seconds * rc_hz)
    delay_steps = round(delay * rc_hz)
    z, vz, thrust = 1.0, start_vz, trim
    measured = [(z, vz)] * (delay_steps + 1)
    heights = []
    for i in range(steps):
        t = i * dt
        seen_z, seen_vz = measured[0]
        e = DroneEstimate(t=t - delay, seq=i, status="detected", valid=True,
                          pos=(0.0, 0.0, seen_z), vel=(0.0, 0.0, seen_vz),
                          pos_std=(0.01, 0.01, 0.01), since_update=0.0)
        ctx = ControlContext(t, dt if i else 0.0, Sticks(throttle=trim),
                             Timestamped(e, t - delay).at(t), TelemetryState())
        if i == 0:
            hold.engage(ctx)
        command = hold.update(ctx).throttle
        thrust += (command - thrust) * min(1.0, dt / 0.05)      # motor lag
        vz += (gain * (thrust - hover) - 0.3 * vz) * dt
        z += vz * dt
        measured = measured[1:] + [(z, vz)]
        heights.append(z)
    return heights


@pytest.mark.parametrize("gain", [30.0, 50.0, 70.0])
@pytest.mark.parametrize("trim", [0.28, 0.30, 0.32])
def test_holds_height_with_the_gain_and_trim_uncertain(gain, trim):
    heights = simulate(gain, hover=0.30, trim=trim)
    tail = heights[-250:]  # the last 5 s
    assert max(abs(h - 1.0) for h in heights) < 0.5     # never trips max_error
    assert max(abs(h - 1.0) for h in tail) < 0.03       # converged
    assert max(tail) - min(tail) < 0.02                 # not oscillating
