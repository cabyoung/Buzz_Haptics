#!/usr/bin/env python3
"""
nav_signals.py - the navigation haptic encoding, on its own.

This file is the encoding and nothing else: no BLE, no serial, no GUI, no
third-party imports. Copy it into any 6-actuator belt codebase and you have the
same signals, with the same timing, as the study it came from.

Belt layout
-----------
6 actuators in one horizontal row. Indices here are 0-based, left to right;
most hardware labels them 1-6, so index i is the motor labelled i+1.

    index  0 1 2 3 4 5
    label  1 2 3 4 5 6
           <-- left     right -->

Signals
-------
Condition 1 - one signal per event, no warning:

    obstacle      all 6 actuators, 3 quick pulses (80 ms on / 150 ms off)
    turn_left     motor 1 alone (left edge)   - position, not a gradient
    turn_right    motor 6 alone (right edge)  - position, not a gradient
    arrive        all 6 actuators, one long buzz (1 s)

A turn is marked by which end of the belt buzzes, in both conditions; condition
2 keeps the same cue and repeats it. The travelling gradient is now used only by
the condition-2 obstacle body.

Condition 2 - every event is announced by a warning, then a pause of
`warning_gap_ms` (the "timer", 2 s by default), then the signal body:

    obstacle_left   warning: one buzz on motor 1 (left edge)
                    body:    turn_right gradient x2
    obstacle_right  warning: one buzz on motor 6 (right edge)
                    body:    turn_left gradient x2
    turn_left       warning: one buzz on motor 1
                    body:    3 more buzzes on motor 1
    turn_right      warning: one buzz on motor 6
                    body:    3 more buzzes on motor 6
    arrive          warning: all 6 actuators buzz
                    body:    one long buzz, the same one condition 1 uses

`obstacle_left` means the obstacle is on your left: the warning buzzes the side
it is on, and the body is the gradient for the *opposite* side, because that is
the way around it. A turn signal, by contrast, warns and turns the same way.

Repeated gradients are separated by `repeat_rest_ms` — one rest, shared by
obstacles and turns, so they stay consistent when you retune it.

How a pattern is represented
----------------------------
`steps()` returns a flat list of two kinds of item, and nothing else:

    Pulse(motors, on_ms, duty)   start these motors now, for on_ms, at duty %
    Wait(ms)                     silence

A Pulse implicitly occupies its own on_ms, so consecutive Pulses run back to
back. This is a plain data structure - you can print it, diff it, or hand it to
your own scheduler without using anything else in this file.

Integration
-----------
Write one function that starts a pulse and returns immediately, then let
`play()` do the timing:

    def pulse(motors, on_ms, duty):        # motors: 0-based indices
        for m in motors:
            my_belt.start(m, on_ms, duty)  # must NOT block for on_ms

    from nav_signals import play, steps
    play(steps('turn_left', condition=1), pulse)
    play(steps('obstacle_left', condition=2), pulse)

If your hardware blocks for the duration of a pulse, pass a `sleep` that
subtracts the time already spent, or set `pulse_blocks=True`:

    play(steps('arrive', condition=2), pulse, pulse_blocks=True)

Every duration is a field on `Params`; nothing is hard-coded at a call site:

    from dataclasses import replace
    slow = replace(DEFAULT_PARAMS, warning_gap_ms=3000)
    play(steps('turn_left', condition=2, params=slow), pulse)

Run `python nav_signals.py` to print every pattern as a timeline, with no
hardware attached.
"""

import time
from dataclasses import dataclass, replace

__all__ = [
    'BELT_SIZE', 'ALL_MOTORS', 'GRADIENT_LEFT', 'GRADIENT_RIGHT',
    'LEFT_EDGE', 'RIGHT_EDGE', 'C1_SIGNALS', 'C2_SIGNALS',
    'Pulse', 'Wait', 'Params', 'DEFAULT_PARAMS',
    'steps', 'warning', 'body', 'play', 'duration_ms', 'timeline', 'describe',
]

BELT_SIZE = 6

ALL_MOTORS = (0, 1, 2, 3, 4, 5)

# A turn gradient travels toward the side being turned to and ends on the
# outermost motor of that side.
GRADIENT_LEFT  = (5, 4, 3, 2, 1, 0)   # motors 6->5->4->3->2->1
GRADIENT_RIGHT = (0, 1, 2, 3, 4, 5)   # motors 1->2->3->4->5->6

# Used by the condition-2 obstacle warning to mark a side spatially.
LEFT_EDGE  = 0    # motor 1
RIGHT_EDGE = 5    # motor 6

C1_SIGNALS = ('obstacle', 'turn_left', 'turn_right', 'arrive')
C2_SIGNALS = ('obstacle_left', 'obstacle_right', 'turn_left', 'turn_right',
              'arrive')


# ── pattern items ─────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Pulse:
    """Start `motors` together for `on_ms` at `duty` %. Occupies on_ms."""
    motors: tuple
    on_ms: int
    duty: int


@dataclass(frozen=True)
class Wait:
    """Silence for `ms`."""
    ms: int


# ── tunable timing ────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Params:
    """Every duration in the encoding. Override with dataclasses.replace()."""

    duty: int = 100                  # all study signals are presented at 100%

    # C1 obstacle: all six, three quick pulses
    obstacle_on_ms:  int = 80
    obstacle_off_ms: int = 150
    obstacle_pulses: int = 3

    # turn gradient: one motor at a time, onsets gradient_step_ms apart. The
    # step is measured onset-to-onset, so changing gradient_on_ms alters how
    # each tap feels without altering how fast the gradient travels.
    gradient_on_ms:   int = 80
    gradient_step_ms: int = 100

    # C1 turn: the edge motor of that side alone, no gradient. A single motor
    # is easy to miss at gradient tap length, so this runs longer than one
    # step of a gradient.
    turn_edge_ms: int = 300

    # C1 arrive: one long buzz on the whole belt
    arrive_ms: int = 1000

    # C2: the "timer" between a warning and the signal it announces
    warning_gap_ms: int = 2000

    # C2 warning buzz - the obstacle side-buzz and the arrive whole-belt buzz.
    # Long enough to be unmissable on its own, short enough not to be confused
    # with the arrival buzz it announces.
    warn_buzz_ms: int = 300

    # C2 bodies: how many gradient passes each event gets. The obstacle body
    # uses the gradient for the side *away* from the obstacle.
    obstacle_repeats: int = 2
    turn_repeats:     int = 3

    # Rest between repeated gradient passes - the same for obstacles and turns
    repeat_rest_ms: int = 200

    # C2 arrive body: one long buzz. Same length as the C1 arrive buzz, so the
    # C2 signal is that buzz with a warning in front of it.
    arrive_long_ms: int = 1000


DEFAULT_PARAMS = Params()


# ── pattern builders ──────────────────────────────────────────────────────────

def _burst(motors, on_ms, off_ms, count, duty):
    """`count` simultaneous pulses on `motors`, on_ms on / off_ms off."""
    out = []
    for i in range(count):
        if i and off_ms > 0:
            out.append(Wait(off_ms))
        out.append(Pulse(tuple(motors), on_ms, duty))
    return out


def _gradient(order, p):
    """Travelling wave along `order`, one motor at a time."""
    out = []
    gap = max(0, p.gradient_step_ms - p.gradient_on_ms)
    for i, m in enumerate(order):
        if i and gap:
            out.append(Wait(gap))
        out.append(Pulse((m,), p.gradient_on_ms, p.duty))
    return out


def _repeat(pattern, count, rest_ms):
    """`count` copies of `pattern`, separated by `rest_ms` of silence."""
    out = []
    for i in range(count):
        if i and rest_ms > 0:
            out.append(Wait(rest_ms))
        out.extend(pattern)
    return out


def _side_of(signal):
    """'left' or 'right', from the signal name."""
    return 'left' if signal.endswith('_left') else 'right'


def _turn_cue(side, p):
    """The turn signal for `side`: that edge motor alone, in both conditions.
    Condition 2 repeats it rather than changing it."""
    edge = LEFT_EDGE if side == 'left' else RIGHT_EDGE
    return [Pulse((edge,), p.turn_edge_ms, p.duty)]


def _turn_gradient(side, p):
    """The gradient that sends the wearer toward `side`. Only the condition-2
    obstacle body uses this now."""
    return _gradient(GRADIENT_LEFT if side == 'left' else GRADIENT_RIGHT, p)


def _away_from(side):
    """The way around an obstacle on `side`."""
    return 'right' if side == 'left' else 'left'


def _check(signal, condition):
    valid = C1_SIGNALS if condition == 1 else C2_SIGNALS
    if condition not in (1, 2):
        raise ValueError(f'condition must be 1 or 2, got {condition!r}')
    if signal in valid:
        return
    if condition == 2 and signal == 'obstacle':
        raise ValueError(
            "condition 2 splits 'obstacle' by side - use 'obstacle_left' or "
            "'obstacle_right'")
    if condition == 1 and signal in ('obstacle_left', 'obstacle_right'):
        raise ValueError(
            f"condition 1 has no side for an obstacle - use 'obstacle' "
            f"(got {signal!r})")
    raise ValueError(f'unknown signal {signal!r} for condition {condition}; '
                     f'expected one of {", ".join(valid)}')


# ── the encoding ──────────────────────────────────────────────────────────────

def warning(signal, condition=2, params=DEFAULT_PARAMS):
    """The warning that announces `signal`. Empty in condition 1."""
    _check(signal, condition)
    if condition == 1:
        return []
    p = params

    if signal.startswith('obstacle'):
        # One buzz on the edge motor of the side the obstacle is on.
        edge = LEFT_EDGE if _side_of(signal) == 'left' else RIGHT_EDGE
        return [Pulse((edge,), p.warn_buzz_ms, p.duty)]

    if signal.startswith('turn'):
        # A single pass of whatever the body will repeat.
        return _turn_cue(_side_of(signal), p)

    # arrive: the whole belt buzzes once
    return [Pulse(ALL_MOTORS, p.warn_buzz_ms, p.duty)]


def body(signal, condition=1, params=DEFAULT_PARAMS):
    """The signal itself, without any warning or gap."""
    _check(signal, condition)
    p = params

    if condition == 1:
        if signal == 'obstacle':
            return _burst(ALL_MOTORS, p.obstacle_on_ms, p.obstacle_off_ms,
                          p.obstacle_pulses, p.duty)
        if signal.startswith('turn'):
            return _turn_cue(_side_of(signal), p)
        return [Pulse(ALL_MOTORS, p.arrive_ms, p.duty)]     # arrive

    if signal.startswith('obstacle'):
        # Turn away from the side the warning marked.
        return _repeat(_turn_gradient(_away_from(_side_of(signal)), p),
                       p.obstacle_repeats, p.repeat_rest_ms)
    if signal.startswith('turn'):
        return _repeat(_turn_cue(_side_of(signal), p),
                       p.turn_repeats, p.repeat_rest_ms)
    return [Pulse(ALL_MOTORS, p.arrive_long_ms, p.duty)]    # arrive


def steps(signal, condition=1, params=DEFAULT_PARAMS):
    """The complete pattern for one presentation of `signal`.

    Condition 1 is just the body. Condition 2 is warning, then a gap of
    params.warning_gap_ms, then the body.
    """
    _check(signal, condition)
    if condition == 1:
        return body(signal, 1, params)
    return (warning(signal, 2, params)
            + [Wait(params.warning_gap_ms)]
            + body(signal, 2, params))


# ── playback ──────────────────────────────────────────────────────────────────

def play(pattern, pulse, sleep=time.sleep, should_stop=None,
         pulse_blocks=False):
    """Run `pattern` on real hardware.

    pulse(motors, on_ms, duty)  starts a pulse and returns immediately; the
                                motors are 0-based indices
    sleep(seconds)              defaults to time.sleep
    should_stop()               optional; checked before every step, and
                                returning True abandons the rest of the pattern
    pulse_blocks                set True if your `pulse` already blocks for
                                on_ms, so it is not waited out twice

    Returns True if the pattern ran to the end, False if it was stopped.

    Steps are not subdivided, so `should_stop` takes effect at the next step
    boundary: asking to stop during the condition-2 timer can take until the
    end of that gap to return. Nothing is buzzing during a Wait, but if you
    need the motors silenced sooner, do that in your own stop path rather than
    waiting for this to return.
    """
    for step in pattern:
        if should_stop is not None and should_stop():
            return False
        if isinstance(step, Pulse):
            pulse(list(step.motors), step.on_ms, step.duty)
            if not pulse_blocks and step.on_ms > 0:
                sleep(step.on_ms / 1000.0)
        elif step.ms > 0:
            sleep(step.ms / 1000.0)
    return True


# ── inspection ────────────────────────────────────────────────────────────────

def duration_ms(pattern):
    """Wall-clock length of a pattern."""
    return sum(s.on_ms if isinstance(s, Pulse) else s.ms for s in pattern)


def timeline(pattern):
    """[(onset_ms, motor_labels, on_ms, duty)] for every pulse in `pattern`.

    Onsets are relative to the start; motor labels are 1-based, as printed on
    the hardware."""
    rows, t = [], 0
    for step in pattern:
        if isinstance(step, Pulse):
            rows.append((t, [m + 1 for m in step.motors], step.on_ms, step.duty))
            t += step.on_ms
        else:
            t += step.ms
    return rows


def describe(signal, condition=1, params=DEFAULT_PARAMS):
    """A printable timeline for one signal."""
    pattern = steps(signal, condition, params)
    head = (f'{signal}  (condition {condition})   '
            f'{len(timeline(pattern))} pulses, '
            f'{duration_ms(pattern) / 1000.0:.2f} s')
    lines = [head, '  ' + '-' * (len(head) - 2)]
    # Count pulses, not steps: the warning also contains Waits.
    warn_len = len(timeline(warning(signal, condition, params)))
    for i, (t, motors, on_ms, duty) in enumerate(timeline(pattern)):
        tag = 'warn' if i < warn_len else ''
        lines.append(f'  {t:>6} ms  motors {str(motors):<22} '
                     f'{on_ms:>5} ms @ {duty:>3}%  {tag}')
    return '\n'.join(lines)


# ── self-test ─────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print(__doc__.split('Belt layout')[0].strip())
    for condition, names in ((1, C1_SIGNALS), (2, C2_SIGNALS)):
        print(f'\n{"=" * 66}\nCONDITION {condition}\n{"=" * 66}')
        for name in names:
            print()
            print(describe(name, condition))

    print(f'\n{"=" * 66}\nOverriding the timer (warning_gap_ms 2000 -> 3500)\n'
          f'{"=" * 66}\n')
    slow = replace(DEFAULT_PARAMS, warning_gap_ms=3500)
    print(describe('turn_left', 2, slow))
