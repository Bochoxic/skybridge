"""Heading fusion: FC yaw for fast turns, the plate against its drift."""

import numpy as np
import pytest

from skybridge import Bus
from skybridge.config import ModeConfig
from skybridge.heading import (
    TOPIC, HeadingFilter, HeadingParams, HeadingTracker, wrap90, wrap180,
)
from skybridge.perception import DroneEstimate
from skybridge.units import us_to_tick

ARM = ModeConfig(aux=1, range_us=(1700, 2100))


def run(truth, t_end, *, bias=0.0, noise=6.0, plates=True, gap=(0, 0), fc_zero=40.0,
        fc_latency=0.05, plate_latency=0.03, outlier_at=None, seed=0, params=None):
    """Feed a filter from a true heading `truth(t)` (left-positive, 0 at arm).

    FC yaw is right-positive, starts at `fc_zero`, drifts by `bias` deg/s and
    arrives `fc_latency` late at 20 Hz. Plate frames (true heading mod 180,
    with noise) are captured at 30 Hz and arrive `plate_latency` late.
    Returns the filter and (t, heading, truth) at every FC sample after arm.
    """
    rng = np.random.default_rng(seed)
    f = HeadingFilter(params or HeadingParams(fc_latency=fc_latency))
    events = []
    for t in np.arange(-0.2, t_end, 1 / 20):
        raw = wrap180(fc_zero - truth(max(t, 0.0)) - bias * max(t, 0.0))
        events.append((t + fc_latency, "fc", raw))
    if plates:
        for t in np.arange(0.0, t_end, 1 / 30):
            if gap[0] <= t < gap[1]:
                continue
            z = truth(t) + rng.normal(0, noise)
            if outlier_at is not None and abs(t - outlier_at) < 1e-6:
                z = truth(t) + 70.0
            events.append((t + plate_latency, "plate", (t, wrap90(z))))
    events.append((0.0, "arm", None))
    events.sort(key=lambda e: (e[0], e[1] != "fc"))
    out = []
    for t_recv, kind, val in events:
        if kind == "arm":
            f.arm()
        elif kind == "fc":
            est = f.on_fc_yaw(t_recv, val)
            if est is not None:
                out.append((t_recv, est.heading_deg, truth(t_recv - fc_latency)))
        else:
            f.on_plate(val[0], val[1], elong=100.0)
    return f, np.array(out)


def turns(t):
    """Left 40 at 5 s, right 70 at 15 s, left 50 at 25 s: fast turns, 60 deg/s."""
    h = 0.0
    for start, delta in ((5, 40), (15, -70), (25, 50)):
        h += delta * min(max((t - start) * 60 / abs(delta), 0.0), 1.0)
    return h


def test_heading_is_zero_at_arm_and_follows_fc_turns():
    f, out = run(turns, 30, plates=False)
    assert out[0, 1] == pytest.approx(0.0, abs=1e-9)
    assert np.max(np.abs(out[:, 1] - out[:, 2])) < 1e-6   # no drift, no plate: FC exactly


def test_left_turn_is_positive():
    f, out = run(lambda t: 30.0 * min(t, 1.0), 2, plates=False)
    assert out[-1, 1] == pytest.approx(30.0, abs=1e-6)


def test_plate_removes_fc_drift():
    f, out = run(turns, 60, bias=0.5)
    late = out[:, 0] > 30
    assert np.max(np.abs(out[late, 1] - out[late, 2])) < 4.0
    assert f.x[1] == pytest.approx(-0.5, abs=0.1)        # drift learned: -(FC bias)
    fc_only = run(turns, 60, bias=0.5, plates=False)[1]
    assert np.abs(fc_only[-1, 1] - fc_only[-1, 2]) > 25  # without the plate, 30 deg off


def test_fast_turns_come_from_fc_without_lag():
    f, out = run(turns, 60, bias=0.5)
    during = (out[:, 0] > 25.05) & (out[:, 0] < 25.8)      # inside the last turn
    assert np.max(np.abs(out[during, 1] - out[during, 2])) < 4.0


def test_coasts_through_a_gap_in_the_plate():
    f, out = run(turns, 60, bias=0.5, gap=(40, 52),
                 noise=6.0, seed=1)
    in_gap = (out[:, 0] > 40) & (out[:, 0] < 52)
    assert np.max(np.abs(out[in_gap, 1] - out[in_gap, 2])) < 4.0  # drift rate carries it


def test_heading_beyond_90_degrees_is_not_folded():
    f, out = run(lambda t: min(t, 3.0) * 50.0, 10, bias=0.3)  # turns to 150 deg
    assert out[-1, 1] == pytest.approx(150.0, abs=4.0)


def test_outlier_plate_frame_is_ignored():
    params = HeadingParams(fc_latency=0.05)
    clean, _ = run(turns, 40, bias=0.5, params=params)
    bad, out = run(turns, 40, bias=0.5, params=params, outlier_at=round(30 * 30) / 30)
    assert bad.rejected == clean.rejected + 1
    assert bad.x[0] == pytest.approx(clean.x[0], abs=0.5)


def test_fc_yaw_wrapping_at_180_is_unwrapped():
    f = HeadingFilter(HeadingParams(fc_latency=0.0))
    f.on_fc_yaw(0.0, 175.0)
    f.arm()
    est = None
    for i, raw in enumerate((178.0, -179.0, -170.0)):       # turning right through 180
        est = f.on_fc_yaw(0.1 * (i + 1), raw)
    assert est.heading_deg == pytest.approx(-15.0)


def test_rearm_starts_again_at_zero():
    f, _ = run(turns, 40, bias=0.5)
    f.disarm()
    assert f.estimate(41.0) is None
    f.arm()
    assert f.estimate(41.0).heading_deg == pytest.approx(0.0)
    assert f.used == 0 and f.x[1] == 0.0


def test_arm_before_any_fc_yaw_takes_the_first_one():
    f = HeadingFilter(HeadingParams(fc_latency=0.0))
    f.arm()
    assert f.estimate(0.0) is None
    assert f.on_fc_yaw(0.1, -60.0).heading_deg == pytest.approx(0.0)
    assert f.on_fc_yaw(0.2, -70.0).heading_deg == pytest.approx(10.0)


def test_plate_before_arm_is_reported_but_not_used():
    f = HeadingFilter()
    f.on_fc_yaw(0.0, 10.0)
    est = f.on_plate(0.0, 12.0, 100.0)
    assert est.heading_deg is None and est.plate_deg == 12.0 and not est.plate_used
    assert f.used == 0


def test_plate_older_than_the_fc_history_is_not_used():
    f = HeadingFilter(HeadingParams(fc_latency=0.0, history=1.0))
    f.on_fc_yaw(10.0, 0.0)
    f.arm()
    f.on_fc_yaw(10.5, 0.0)
    assert f.on_plate(9.0, 5.0, 100.0).heading_deg is None


def test_low_confidence_plate_frames_move_the_offset_less():
    strong, weak = HeadingFilter(HeadingParams(fc_latency=0.0)), HeadingFilter(
        HeadingParams(fc_latency=0.0))
    for f, elong in ((strong, 100.0), (weak, 2.0)):
        f.on_fc_yaw(0.0, 0.0)
        f.arm()
        f.on_fc_yaw(0.05, 0.0)
        f.on_plate(0.05, 20.0, elong)
    assert strong.x[0] > 2 * weak.x[0] > 0


# ── On the Bus ────────────────────────────────────────────────────────────────
def ticks(arm_us):
    t = [us_to_tick(1500)] * 16
    t[ARM.channel] = us_to_tick(arm_us)
    return tuple(t)


def test_tracker_publishes_heading_from_bus_messages():
    bus = Bus()
    got = []
    bus.subscribe(TOPIC, lambda topic, est, t: got.append(est))
    tracker = HeadingTracker(bus, ARM, HeadingParams(fc_latency=0.0))
    tracker.start()
    try:
        bus.publish("rc/sent", ticks(988), 0.0)
        bus.publish("telemetry/attitude", {"yaw_deg": 10.0, "roll_deg": 0.0}, 0.0)
        assert got == []                                    # not armed: nothing to say
        bus.publish("rc/sent", ticks(2012), 0.1)
        bus.publish("telemetry/attitude", {"yaw_deg": 5.0}, 0.2)   # turned right 5
        assert got[-1].heading_deg == pytest.approx(5.0)    # ref is 10 at arm: -(5-10)
        e = DroneEstimate(t=0.2, seq=1, status="detected", valid=True,
                          plate_yaw=7.0, plate_elong=80.0)
        bus.publish("perception/estimate", e, 0.21)
        assert got[-1].plate_deg == pytest.approx(7.0) and got[-1].plate_used
        assert tracker.latest is got[-1]
        bus.publish("perception/estimate",
                    DroneEstimate(t=0.25, seq=2, status="detected", valid=True), 0.26)
        assert got[-1].plate_deg == pytest.approx(7.0)       # no plate: nothing published
        bus.publish("rc/sent", ticks(988), 0.3)
        assert tracker.latest is None
    finally:
        tracker.close()


def test_tracker_stops_listening_when_closed():
    bus = Bus()
    tracker = HeadingTracker(bus, ARM)
    tracker.start()
    tracker.close()
    bus.publish("rc/sent", ticks(2012), 0.0)
    assert not tracker.filter.armed
