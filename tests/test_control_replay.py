"""Running controllers over recorded flights."""

import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from skybridge import Bus, DroneConfig
from skybridge.config import ModeConfig
from skybridge.control import AltitudeHold, AltitudeHoldParams
from skybridge.control.replay import (
    FlightLog,
    _reason,
    load_flight_log,
    run_replay,
    summarize,
)
from skybridge.perception import session as session_module
from skybridge.perception.link import PerceptionLink
from skybridge.perception.session import PerceptionSession
from skybridge.perception.types import DroneEstimate
from skybridge.units import throttle_to_tick

ROOT = Path(__file__).parent.parent
ARM = ModeConfig(aux=1, range_us=(1700, 2100))
ARMED, DISARMED = 1900.0, 1000.0


def us(fraction):
    return 1000.0 + fraction * 1000.0


def flight(seconds=4.0, z=lambda t: 1.0, throttle=lambda t: 0.3, arm=lambda t: ARMED,
           since=lambda t: 0.0, keep=lambda t: True):
    rc_t = np.arange(0.0, seconds, 0.02)
    all_est = np.arange(0.0, seconds, 1.0 / 30.0)
    est_t = np.array([t for t in all_est if keep(t)])
    centre = np.full(rc_t.size, 1500.0)
    return FlightLog(
        rc_t=rc_t,
        rc_us={"roll": centre, "pitch": centre, "yaw": centre,
               "throttle": np.array([us(throttle(t)) for t in rc_t]),
               "arm": np.array([arm(t) for t in rc_t])},
        est_t=est_t,
        pos=np.array([[0.0, 0.0, z(t)] for t in est_t]).reshape(-1, 3),
        vel=np.zeros((est_t.size, 3)),
        pos_std=np.full((est_t.size, 3), 0.01),
        since_update=np.array([since(t) for t in est_t]),
    )


def hold(**params):
    return AltitudeHold(AltitudeHoldParams(**params))


def test_target_and_trim_default_to_the_medians():
    log = flight(z=lambda t: 1.0 if t < 3.0 else 1.2, throttle=lambda t: 0.3 if t < 3.0 else 0.4)
    result = run_replay(log, hold(), ARM)
    assert result.setpoint == pytest.approx(1.0)
    assert result.trim == pytest.approx(0.3, abs=0.002)


def test_explicit_target_and_trim_are_used():
    result = run_replay(flight(), hold(), ARM, setpoint=1.1, trim=0.35)
    assert result.setpoint == 1.1
    assert result.trim == 0.35
    assert result.debug["setpoint"][-1] == pytest.approx(1.1)


def test_output_opposes_the_height_error():
    log = flight(z=lambda t: 0.9 if t < 2.0 else 1.1)
    result = run_replay(log, hold(ki=0.0), ARM, setpoint=1.0, trim=0.3)
    assert result.output[(result.t > 0.5) & (result.t < 1.5)].min() > 0.3
    assert result.output[result.t > 2.5].max() < 0.3


def test_only_estimates_that_had_arrived_are_used():
    log = flight(z=lambda t: 1.0 + t * 0.01)
    result = run_replay(log, hold(), ARM, latency=0.2)
    assert np.isnan(result.z[0])
    assert result.dropouts["no track"] > 0
    k = np.searchsorted(result.t, 2.0)
    capture = log.est_t[np.searchsorted(log.est_t + 0.2, result.t[k], side="right") - 1]
    assert result.z[k] == pytest.approx(1.0 + capture * 0.01)


def test_track_gap_stops_output_then_it_resumes():
    log = flight(keep=lambda t: not 1.0 < t < 2.0)
    result = run_replay(log, hold(), ARM)
    gap = (result.t > 1.5) & (result.t < 2.0)
    assert np.isnan(result.output[gap]).all()
    assert result.dropouts["estimate stale"] > 0
    assert np.isfinite(result.output[result.t > 2.5]).all()


def test_long_coasting_counts_as_a_stop():
    log = flight(since=lambda t: 0.5 if 1.0 < t < 2.0 else 0.0)
    result = run_replay(log, hold(), ARM)
    assert result.dropouts["track coasting"] > 0


def test_only_airborne_ticks_are_run():
    log = flight(arm=lambda t: DISARMED if t < 1.0 else ARMED,
                 throttle=lambda t: 0.05 if t > 3.0 else 0.3)
    result = run_replay(log, hold(), ARM)
    assert result.t.min() >= 1.0
    assert result.t.max() <= 3.0


def test_no_airborne_ticks_is_an_error():
    with pytest.raises(ValueError, match="airborne"):
        run_replay(flight(arm=lambda t: DISARMED), hold(), ARM)


def test_no_track_while_airborne_is_an_error():
    with pytest.raises(ValueError, match="no tracked height"):
        run_replay(flight(keep=lambda t: False), hold(), ARM)


def test_never_running_is_an_error():
    log = flight(z=lambda t: 5.0)
    result = run_replay(log, hold(), ARM, setpoint=1.0)
    with pytest.raises(ValueError, match="never ran"):
        summarize(result)


@pytest.mark.parametrize("message, reason", [
    ("height error +0.52 m", "height error"),
    ("estimate stale (0.25 s)", "estimate stale"),
    ("no track", "no track"),
])
def test_reasons_are_grouped_without_their_numbers(message, reason):
    assert _reason(message) == reason


def test_summary_compares_with_the_pilot():
    log = flight(z=lambda t: 0.9, throttle=lambda t: 0.3)
    stats = summarize(run_replay(log, hold(ki=0.0), ARM, setpoint=1.0, trim=0.3,
                                 latency=0.0))
    assert stats.running == 1.0
    assert stats.mean_diff == pytest.approx(0.008, abs=0.002)  # kp * 0.1
    assert stats.correlation == 0.0  # the pilot's stick never moved
    assert math.isfinite(stats.rms_diff)


def write_log(tmp_path, monkeypatch):
    """A flight log written by the real recording path."""
    monkeypatch.setattr(session_module, "LOG_DIR", tmp_path)
    bus = Bus()
    drone = DroneConfig.from_dict({"modes": {"arm": {"aux": 1, "range": [1700, 2100]}}})
    session = PerceptionSession(bus, drone, viewer=False, record=True)
    session._start_logging()
    link = PerceptionLink("127.0.0.1", 0, bus)
    start = time.monotonic()
    ticks = [992] * 16
    ticks[2] = throttle_to_tick(0.3)
    ticks[4] = 1811
    for i in range(100):
        bus.publish("rc/sent", tuple(ticks), start + i * 0.02)
    for i in range(60):
        link.receive(DroneEstimate(
            t=start + i / 30.0, seq=i, status="detected", valid=True,
            pos=(0.0, 0.0, 0.5), vel=(0.0, 0.0, 0.1), pos_std=(0.02, 0.02, 0.02),
            since_update=0.0).to_wire())
    link.close()
    session.close()
    return tmp_path / f"{session.run_id}_flight.rrd"


def test_loads_a_log_from_the_real_recording_path(tmp_path, monkeypatch):
    log = load_flight_log(write_log(tmp_path, monkeypatch))
    assert log.rc_t.size == 100
    assert log.est_t.size == 60
    assert np.all(np.diff(log.est_t) > 0)
    assert log.pos[0, 2] == pytest.approx(0.5)
    assert log.vel[0, 2] == pytest.approx(0.1)
    assert set(log.rc_us) >= {"roll", "pitch", "throttle", "yaw", "arm"}
    assert run_replay(log, hold(), ARM).setpoint == pytest.approx(0.5)


def test_script_runs_end_to_end(tmp_path, monkeypatch):
    log = write_log(tmp_path, monkeypatch)
    out = tmp_path / "plots.rrd"
    done = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "control_replay.py"), str(log),
         "--save", str(out), "--set", "kp=0.1"],
        capture_output=True, text=True, timeout=60, check=False,
    )
    assert done.returncode == 0, done.stderr
    assert "controller - pilot" in done.stdout
    assert out.stat().st_size > 0


def test_script_rejects_a_bad_param(tmp_path, monkeypatch):
    log = write_log(tmp_path, monkeypatch)
    done = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "control_replay.py"), str(log),
         "--no-viewer", "--set", "kpp=0.1"],
        capture_output=True, text=True, timeout=60, check=False,
    )
    assert done.returncode != 0
    assert "kpp" in done.stderr
