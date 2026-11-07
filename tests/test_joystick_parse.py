"""joydev event parsing and JoystickSource state handling, without a device."""

import struct

from skybridge.bus import Bus
from skybridge.rc.joystick import (
    JS_EVENT_AXIS,
    JS_EVENT_BUTTON,
    JS_EVENT_INIT,
    JoystickSource,
    parse_js_events,
)


def ev(value, typ, number, t=0):
    return struct.pack("IhBB", t, value, typ, number)


def test_parses_complete_events_and_keeps_partial_tail():
    buf = bytearray(ev(-32767, JS_EVENT_AXIS | JS_EVENT_INIT, 2) + ev(1, JS_EVENT_BUTTON, 3))
    buf += ev(5, JS_EVENT_AXIS, 0)[:3]

    events = parse_js_events(buf)

    assert [(e.value, e.number, e.is_axis, e.is_init) for e in events] == [
        (-32767, 2, True, True),
        (1, 3, False, False),
    ]
    assert events[1].is_button
    assert len(buf) == 3


def test_partial_event_completes_on_next_read():
    buf = bytearray(ev(123, JS_EVENT_AXIS, 1)[:5])
    assert parse_js_events(buf) == []
    buf += ev(123, JS_EVENT_AXIS, 1)[5:]
    (e,) = parse_js_events(buf)
    assert e.value == 123 and e.number == 1


def bare_source(num_axes=3, num_buttons=1, bus=None):
    """A JoystickSource with no device or thread, to drive _apply directly."""
    src = JoystickSource.__new__(JoystickSource)
    import threading

    src._bus = bus
    src._clock = lambda: 42.0
    src.num_axes, src.num_buttons = num_axes, num_buttons
    src._lock = threading.Lock()
    src._axes, src._buttons = [0] * num_axes, [0] * num_buttons
    src._axes_seen = set()
    src._frame = None
    src._alive = True
    return src


def test_no_frame_until_every_axis_initialised():
    # An uninitialised axis reads 0 = mid stick: 50% throttle if it's CH3.
    src = bare_source()
    src._apply(parse_js_events(bytearray(ev(0, JS_EVENT_AXIS | JS_EVENT_INIT, 0))))
    assert src.latest() is None

    src._apply(parse_js_events(bytearray(
        ev(0, JS_EVENT_AXIS | JS_EVENT_INIT, 1) + ev(-32767, JS_EVENT_AXIS | JS_EVENT_INIT, 2)
    )))
    frame = src.latest()
    assert frame.axes == (0, 0, -32767)
    assert frame.received == 42.0


def test_updates_publish_on_bus():
    bus, seen = Bus(), []
    bus.subscribe("rc/", lambda topic, payload, t: seen.append((topic, payload.axes, t)))
    src = bare_source(num_axes=1, bus=bus)

    src._apply(parse_js_events(bytearray(ev(100, JS_EVENT_AXIS | JS_EVENT_INIT, 0))))
    src._apply(parse_js_events(bytearray(ev(200, JS_EVENT_AXIS, 0))))

    assert seen == [("rc/input", (100,), 42.0), ("rc/input", (200,), 42.0)]


def test_out_of_range_numbers_are_ignored():
    src = bare_source(num_axes=1, num_buttons=1)
    src._apply(parse_js_events(bytearray(
        ev(7, JS_EVENT_AXIS | JS_EVENT_INIT, 0) + ev(9, JS_EVENT_AXIS, 5) + ev(1, JS_EVENT_BUTTON, 8)
    )))
    assert src.latest().axes == (7,)
    assert src.latest().buttons == (0,)
