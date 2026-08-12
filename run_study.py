#!/usr/bin/env python3
"""
run_study.py — plays a fixed study block of haptic signals, one at a time.

Each block is a pre-randomized presentation order for one condition/set. The
signals come from nav_signals.py via haptics_nav.NavHaptics, so the encoding
here is identical to the scripted demo and the CLI console — this script only
decides *which* signal fires and *when*.

Conditions
----------
C1  4 signals, no warning:
        obstacle, turn_left, turn_right, arrive

C2  5 signals, each announced by a warning and a timer (--timer, 2 s default):
        obstacle_left    buzz on motor 1    -> timer -> right gradient x2
        obstacle_right   buzz on motor 6    -> timer -> left gradient x2
        turn_left        one buzz on motor 1 -> timer -> 3 more buzzes
        turn_right       one buzz on motor 6 -> timer -> 3 more buzzes
        arrive           all 6 buzz         -> timer -> one long buzz

    An obstacle warning buzzes the side the obstacle is on and the body then
    sends the wearer the other way; a turn warns and turns the same way. C2
    splits `obstacle` by side, so a block is 10 trials rather than 8.

Blocks
------
    c1s1, c1s2   condition 1, 8 trials  (each of 4 signals twice)
    c2s1, c2s2   condition 2, 10 trials (each of 5 signals twice)

Every block is block-randomized: each half is its own shuffle of the full
signal set, so a signal appears once per half and position is balanced, but the
second half is never a copy of the first. No signal repeats back to back, and
the two sets in a condition share at most one position. `--shuffle SEED`
redraws them under the same constraints, reproducibly.

Usage
-----
    python run_study.py c1s1                    # one block, BLE backend
    python run_study.py c2s1 c2s2               # two blocks, back to back
    python run_study.py c1s1 --backend dry      # no hardware, print only
    python run_study.py c1s1 --gap 5            # 5 s between signals
    python run_study.py c2s1 --timer 3          # 3 s warning -> signal gap
    python run_study.py c2s1 --list             # print the order, fire nothing
    python run_study.py c1s1 --shuffle 7 --list # fresh order for participant 7
"""

import argparse
import random
import sys
import time
from dataclasses import replace

try:
    from haptics_nav import NavHaptics                         # run as a script
    import nav_signals as sig
except ImportError:
    from buzzHaptics.Buzz_Haptics.haptics_nav import NavHaptics  # run as a package
    from buzzHaptics.Buzz_Haptics import nav_signals as sig

GAP = 5.0  # seconds between consecutive signals

DEFAULT_SEED = 20260810   # the seed the fixed orders below were drawn from

# Presentation orders, drawn by generate_blocks(DEFAULT_SEED) and written out
# here so the exact sequences a participant saw are readable in the source and
# cannot shift with a Python version. --shuffle SEED redraws them.
#
# Every block: each signal twice, block-randomized (each half is its own
# shuffle of the full set, so a signal appears once per half), never the same
# signal back to back, the two halves never identical, and the two sets in a
# condition share at most one position.
BLOCKS = {
    'c1s1': ['turn_left', 'turn_right', 'obstacle', 'arrive',
             'turn_right', 'turn_left', 'obstacle', 'arrive'],
    'c1s2': ['turn_right', 'arrive', 'obstacle', 'turn_left',
             'arrive', 'obstacle', 'turn_right', 'turn_left'],
    'c2s1': ['obstacle_left', 'obstacle_right', 'turn_right', 'arrive',
             'turn_left', 'arrive', 'obstacle_left', 'turn_left',
             'obstacle_right', 'turn_right'],
    'c2s2': ['arrive', 'turn_left', 'obstacle_left', 'obstacle_right',
             'turn_right', 'turn_left', 'arrive', 'obstacle_right',
             'turn_right', 'obstacle_left'],
}

BLOCK_ORDER = ['c1s1', 'c1s2', 'c2s1', 'c2s2']


# ── randomization ─────────────────────────────────────────────────────────────

def random_block(signals, repeats, rng, tries=10000):
    """One block-randomized presentation order.

    Each repetition is an independent shuffle of the full signal set, so every
    signal lands once per half and position is balanced against learning and
    fatigue — but the halves are not copies of each other, which is what makes
    a repeated-block order predictable. Orders with a signal repeated back to
    back, including across the half boundary, are rejected.
    """
    signals = list(signals)
    for _ in range(tries):
        halves = []
        for _r in range(repeats):
            half = signals[:]
            rng.shuffle(half)
            halves.append(half)
        if any(halves[i] == halves[j]
               for i in range(len(halves)) for j in range(i + 1, len(halves))):
            continue                                   # a half repeats another
        order = [s for half in halves for s in half]
        if any(order[i] == order[i - 1] for i in range(1, len(order))):
            continue                                   # back-to-back repeat
        return order
    raise RuntimeError('could not satisfy the randomization constraints')


def generate_blocks(seed, repeats=2):
    """All four blocks for `seed`. Deterministic: the same seed always gives
    the same orders, so a participant's block can be reproduced from their
    number alone."""
    rng = random.Random(seed)
    blocks = {}
    for cond, names in ((1, sig.C1_SIGNALS), (2, sig.C2_SIGNALS)):
        for set_n in (1, 2):
            first = blocks.get(f'c{cond}s1')
            while True:
                order = random_block(names, repeats, rng)
                if first is None:
                    break
                # The second set must not shadow the first: at most one trial
                # may hold the same signal in the same position.
                if (order[0] != first[0]
                        and sum(a == b for a, b in zip(order, first)) <= 1):
                    break
            blocks[f'c{cond}s{set_n}'] = order
    return blocks


def condition_of(block: str) -> int:
    """c1s1 -> 1, c2s2 -> 2."""
    return int(block[1])


def print_block(name, params):
    signals = BLOCKS[name]
    cond    = condition_of(name)
    total   = sum(sig.duration_ms(sig.steps(s, cond, params)) for s in signals)
    print(f'\n  {name.upper()}  ({len(signals)} signals, condition {cond}, '
          f'{total / 1000.0:.1f} s of haptics)')
    print('  ' + '-' * 52)
    for i, name_ in enumerate(signals, start=1):
        pattern = sig.steps(name_, cond, params)
        print(f'  {i:>2}.  {name_:<16} {len(sig.timeline(pattern)):>2} pulses  '
              f'{sig.duration_ms(pattern) / 1000.0:>5.2f} s')


def build_nav(args, params):
    """Connect the requested backend. Returns (None, None) if the hardware is
    unavailable and the caller has not opted into running without it."""
    backend = 'dry' if args.dry_run else args.backend
    try:
        return NavHaptics(backend=backend, port=args.port, name=args.name,
                          address=args.address, params=params), backend
    except Exception as exc:
        if backend == 'dry':
            raise
        print(f'\n! {backend} backend unavailable ({exc})')
        if not args.allow_dry:
            # Falling back silently would run a whole block past a participant
            # with the belt doing nothing, and the printout looks like a normal
            # session. Refuse instead.
            print('  REFUSING to run a block with no haptics - the belt would '
                  'stay silent while\n  the trials scrolled past as if they had '
                  'been presented.')
            print('  Power-cycle the belt and try again, or pass --allow-dry to '
                  'rehearse\n  the block with no hardware.\n')
            return None, None
        print('  --allow-dry given: rehearsing with no hardware, '
              'signals will print but not actuate\n')
        return NavHaptics(backend='dry', params=params), 'dry'


def run_block(nav, name, gap, first):
    """Play one block. `first` is False once anything has already fired, so the
    inter-signal gap is also honoured across a block boundary."""
    signals = BLOCKS[name]
    cond    = condition_of(name)
    print(f'\n=== {name.upper()} - condition {cond}, '
          f'{len(signals)} signals, {gap:g} s apart ===\n')

    for i, signal in enumerate(signals, start=1):
        if not first:
            time.sleep(gap)
        first = False
        secs = sig.duration_ms(nav.steps(signal, cond)) / 1000.0
        print(f'  {i:>2}/{len(signals)}  >>> {signal:<16} ({secs:.2f} s)')
        nav.play(signal, cond)

    return first


def main():
    p = argparse.ArgumentParser(
        description='Play a study block of haptic signals.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('blocks', nargs='+', metavar='BLOCK',
                   choices=BLOCK_ORDER,
                   help='one or more of: ' + ', '.join(BLOCK_ORDER)
                        + ' (played in the order given)')
    p.add_argument('--gap', type=float, default=GAP,
                   help=f'seconds between signals (default: {GAP:g})')
    p.add_argument('--timer', type=float,
                   default=sig.DEFAULT_PARAMS.warning_gap_ms / 1000.0,
                   help='condition 2: seconds between a warning and the signal '
                        'it announces (default: '
                        f'{sig.DEFAULT_PARAMS.warning_gap_ms / 1000.0:g})')
    p.add_argument('--backend', choices=['ble', 'serial', 'dry'], default='ble',
                   help='ble = nRF5340 device, serial = Arduino belt, dry = no hardware')
    p.add_argument('--name', default='HapticMotor', help='BLE advertised name')
    p.add_argument('--address', default=None, help='BLE address (overrides --name)')
    p.add_argument('--port', default='COM3', help='serial port (default: COM3)')
    p.add_argument('--dry-run', action='store_true', help='alias for --backend dry')
    p.add_argument('--allow-dry', action='store_true',
                   help='if the belt cannot be reached, rehearse the block with '
                        'no hardware instead of refusing to run')
    p.add_argument('--shuffle', type=int, metavar='SEED', default=None,
                   help='redraw the presentation orders from SEED (e.g. the '
                        'participant number) instead of using the fixed ones; '
                        'same constraints, and the same seed always gives the '
                        'same orders')
    p.add_argument('--list', action='store_true', dest='list_only',
                   help='print the presentation order and exit')
    args = p.parse_args()

    params = replace(sig.DEFAULT_PARAMS,
                     warning_gap_ms=int(round(args.timer * 1000)))

    if args.shuffle is not None:
        BLOCKS.update(generate_blocks(args.shuffle))
        print(f'\n[shuffle] orders redrawn from seed {args.shuffle}')

    if args.list_only:
        for name in args.blocks:
            print_block(name, params)
        print()
        return 0

    nav, backend = build_nav(args, params)
    if nav is None:
        return 1
    print(f'\nStudy runner - backend: {backend}   '
          f'blocks: {" ".join(args.blocks)}   gap: {args.gap:g} s   '
          f'timer: {args.timer:g} s')

    first = True
    try:
        for name in args.blocks:
            first = run_block(nav, name, args.gap, first)
        print('\nBlock complete.\n')
    except KeyboardInterrupt:
        # Ctrl-C mid-block must not leave a motor latched on.
        nav.stop()
        print('\n[ABORTED] stopped by user')
    except Exception as exc:
        print(f'\n[ERROR] run aborted: {exc}')
        return 1
    finally:
        try:
            nav.close()
        except Exception:
            pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
