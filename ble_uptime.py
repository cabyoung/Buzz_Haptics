#!/usr/bin/env python3
"""
ble_uptime.py — how long does the belt stay connected?

Connects, holds the link, and times how long it survives. Repeat it a few times
and you get a distribution rather than an anecdote, which is what you need to
tell a flaky radio from a dying battery.

The point is the three modes, which fail for different reasons:

    --idle      connect and touch nothing. The link is held open by the BLE
                stack alone. A drop here is RF, firmware, or a connection
                supervision timeout - nothing to do with your traffic.

    --ping      write a silent OFF every --interval seconds (default). Same
                GATT path the study uses, but no motor ever runs, so no current
                is drawn. A drop *only* in this mode points at the BLE stack or
                the write path, not at power.

    --buzz      pulse all six motors every --interval seconds. This is the
                worst-case current draw. A link that survives --ping for
                minutes but drops within seconds of --buzz is almost certainly
                browning out: battery sagging under motor load until the radio
                resets. Check the battery and the supply rail before you
                suspect the firmware.

Run all three before concluding anything. "It disconnects" means something very
different in each.

--ramp turns the answer into a number. It buzzes 1 motor, then 2, then 3 ... and
reports the count at which the link breaks. Multiply that by the per-motor
current and compare it against what the regulator can actually deliver: if the
board runs its motors off a TPS7A02-class LDO (200 mA, with overcurrent
protection), breaking somewhere around 2-3 motors is the regulator limiting and
pulling the rail down, not a radio fault.

Usage
-----
    python ble_uptime.py                     # one trial, ping mode, hold 120 s
    python ble_uptime.py --trials 5          # five connect/hold cycles + stats
    python ble_uptime.py --idle              # no traffic at all
    python ble_uptime.py --buzz              # motors running, worst-case power
    python ble_uptime.py --ramp              # find how many motors it survives
    python ble_uptime.py --buzz --duty 60    # same load, lower duty cycle
    python ble_uptime.py --max-hold 0        # hold until it drops, no time limit
    python ble_uptime.py --interval 0.25     # hammer it four times a second
    python ble_uptime.py --scan              # list devices and exit
    python ble_uptime.py --address AA:BB:CC:DD:EE:FF
    python ble_uptime.py --trials 10 --csv uptime.csv

Ctrl-C ends the current trial and still prints the summary.
"""

import argparse
import csv
import os
import statistics
import sys
import time
from datetime import datetime

try:
    from haptic_ble import (                                   # run as a script
        BleHapticLink, BELT_SIZE, DEFAULT_NAME)
except ImportError:
    from buzzHaptics.Buzz_Haptics.haptic_ble import (          # run as a package
        BleHapticLink, BELT_SIZE, DEFAULT_NAME)

MAX_HOLD  = 120.0   # seconds before a trial is called a pass and stopped
INTERVAL  = 1.0     # seconds between pings / buzzes
BUZZ_MS   = 150     # pulse length in --buzz mode
SETTLE    = 3.0     # seconds between trials, so the stack can clean up
POLL      = 0.05    # how often the hold loop wakes up


def stamp():
    return datetime.now().strftime('%H:%M:%S')


def print_scan(timeout, match=DEFAULT_NAME):
    """List every peripheral in range, strongest signal first.

    Kept local so this tool depends on nothing but BleHapticLink - when you are
    chasing a hardware fault you want the diagnostic to have as few moving
    parts as possible. RSSI tells identical belts apart: the nearer one reads
    higher, closer to zero."""
    import asyncio
    from bleak import BleakScanner

    async def _scan():
        found = await BleakScanner.discover(timeout=timeout, return_adv=True)
        return [((adv.local_name or dev.name), dev.address, adv.rssi)
                for dev, adv in found.values()]

    print(f'[BLE] Scanning {timeout:g} s ...')
    rows = sorted(asyncio.run(_scan()), key=lambda r: (r[2] is None, -(r[2] or 0)))
    if not rows:
        print('  nothing found - is the belt powered and advertising?\n')
        return rows

    print(f'\n  {"name":<28} {"address":<20} {"rssi":>5}')
    print('  ' + '-' * 57)
    for name, address, rssi in rows:
        hit = match and name and match.lower() in name.lower()
        print(f'  {(name or "(unnamed)"):<28} {address:<20} '
              f'{rssi if rssi is not None else "?":>5}'
              f'{"  <-- match" if hit else ""}')
    print(f'\n  Target one with:  --address <address>\n')
    return rows


def hold(link, args, motors):
    """Hold the link until it drops or --max-hold is reached.

    Returns (seconds_held, reason, writes). A write failing is the ground
    truth here - `is_connected` can lag behind reality on Windows, so traffic
    modes trust the exception and only fall back to the flag when idle."""
    t0        = time.perf_counter()
    next_ping = t0
    writes    = 0

    while True:
        now  = time.perf_counter()
        held = now - t0

        if args.max_hold and held >= args.max_hold:
            return held, f'still up at the {args.max_hold:g} s limit', writes

        if not link.connected:
            return held, 'link reported disconnected', writes

        if args.mode != 'idle' and now >= next_ping:
            try:
                if args.mode == 'buzz':
                    link.pulse_many(list(range(motors)), args.buzz_ms, args.duty)
                else:
                    link.off(0)          # silent: stops one motor, no vibration
                writes += 1
            except Exception as exc:
                return time.perf_counter() - t0, f'write failed: {exc}', writes
            next_ping = now + args.interval

        time.sleep(POLL)


def trial(n, args, target, motors):
    """One connect/hold/disconnect cycle. Returns a result dict."""
    load = f' [{motors} motor{"s" if motors != 1 else ""} @ {args.duty}%]' \
           if args.mode == 'buzz' else ''
    print(f'\ntrial {n}{load}  {stamp()}  connecting to {target} ...')
    row = {'trial': n, 'started': datetime.now().isoformat(timespec='seconds'),
           'mode': args.mode, 'motors': motors if args.mode == 'buzz' else 0,
           'duty': args.duty if args.mode == 'buzz' else 0,
           'connected': False, 'connect_s': None,
           'uptime_s': None, 'writes': 0, 'reason': ''}

    t0 = time.perf_counter()
    try:
        # attempts=1: this is a stability test, so a failure must be reported,
        # not quietly retried the way a study run would.
        link = BleHapticLink(name=args.name, address=args.address).connect(attempts=1)
    except Exception as exc:
        row['connect_s'] = time.perf_counter() - t0
        row['reason']    = f'connect failed: {exc}'
        print(f'          {stamp()}  FAILED to connect after '
              f'{row["connect_s"]:.1f} s - {exc}')
        return row

    row['connected'] = True
    row['connect_s'] = time.perf_counter() - t0
    print(f'          {stamp()}  connected in {row["connect_s"]:.1f} s, '
          f'holding in {args.mode} mode ...')

    try:
        held, reason, writes = hold(link, args, motors)
    except KeyboardInterrupt:
        held, reason, writes = time.perf_counter() - t0, 'stopped by user', 0
        print()
    finally:
        try:
            link.close()
        except Exception:
            pass

    row['uptime_s'], row['reason'], row['writes'] = held, reason, writes
    verdict = 'HELD' if 'still up' in reason else 'DROPPED'
    print(f'          {stamp()}  {verdict} after {held:.1f} s '
          f'({writes} writes) - {reason}')
    return row


def ramp_verdict(rows, args):
    """Where in the motor-count ramp did it break, and what current does that
    imply? This is the number that tells a supply problem from a radio one."""
    print('\n  ramp result')
    print('  ' + '-' * 64)
    survived, failed = [], []
    for r in rows:
        est = r['motors'] * args.ma_per_motor * args.duty / 100.0
        ok  = r['connected'] and 'still up' in r['reason']
        (survived if ok else failed).append(r['motors'])
        print(f'  {r["motors"]} motor{"s " if r["motors"] != 1 else "  "}'
              f'~{est:>4.0f} mA est   '
              f'{"held" if ok else "FAILED"}  {r["reason"]}')

    print()
    if not failed:
        print('  Survived every motor count. The supply is not the limit here - '
              'look at\n  RF, firmware, or the connection supervision timeout '
              'instead.')
        return
    if not survived:
        print('  Failed even at one motor. That is not a current-budget '
              'problem; something\n  more basic is wrong with the link or the '
              'board.')
        return

    threshold = min(failed)
    est = threshold * args.ma_per_motor * args.duty / 100.0
    print(f'  Breaks at {threshold} motors, roughly {est:.0f} mA by the '
          f'{args.ma_per_motor} mA/motor estimate.')
    print(f'  A TPS7A02-class LDO is rated 200 mA with overcurrent protection, '
          f'so a\n  break anywhere near that is the regulator current-limiting '
          f'and dragging\n  the rail down, not a radio fault. Confirm with a '
          f'scope on the rail while\n  the belt buzzes; a visible droop settles '
          f'it.')


def summarize(rows, args):
    print('\n' + '=' * 68)
    print('SUMMARY')
    print('=' * 68)

    load_col = args.mode == 'buzz'
    head = f'\n  {"#":<3} ' + (f'{"motors":>7} ' if load_col else '')
    print(head + f'{"connect":>9} {"uptime":>9} {"writes":>7}  outcome')
    print('  ' + '-' * 68)
    for r in rows:
        conn = f'{r["connect_s"]:.1f} s' if r['connect_s'] is not None else '-'
        up   = f'{r["uptime_s"]:.1f} s' if r['uptime_s'] is not None else '-'
        load = f'{r["motors"]:>7} ' if load_col else ''
        print(f'  {r["trial"]:<3} {load}{conn:>9} {up:>9} {r["writes"]:>7}  '
              f'{r["reason"]}')

    if args.ramp:
        ramp_verdict(rows, args)

    ok       = [r for r in rows if r['connected']]
    connects = [r['connect_s'] for r in ok]
    uptimes  = [r['uptime_s'] for r in ok if r['uptime_s'] is not None]
    held     = [r for r in ok if 'still up' in r['reason']]

    print(f'\n  connected        {len(ok)}/{len(rows)} attempts', end='')
    print(f'  ({100 * len(ok) / len(rows):.0f}%)' if rows else '')
    if connects:
        print(f'  connect time     min {min(connects):.1f}   '
              f'median {statistics.median(connects):.1f}   '
              f'max {max(connects):.1f} s')
    if uptimes:
        print(f'  uptime           min {min(uptimes):.1f}   '
              f'median {statistics.median(uptimes):.1f}   '
              f'max {max(uptimes):.1f} s')
        print(f'  reached the {args.max_hold:g} s limit   '
              f'{len(held)}/{len(uptimes)}'
              if args.max_hold else
              f'  no time limit was set')

    # The interpretation is the whole point of the script, so spell it out.
    print()
    if not ok:
        print('  Never connected. The belt is probably not advertising - '
              'check power,\n  then `python ble_uptime.py --scan` to see if it '
              'is on the air at all.')
    elif uptimes and all('still up' in r['reason'] for r in ok):
        print(f'  Stable in {args.mode} mode for the whole test. Re-run with '
              f'the other\n  modes (--idle / --ping / --buzz) before calling it '
              f'healthy - only\n  --buzz loads the battery.')
    elif uptimes:
        worst = min(uptimes)
        print(f'  Dropping in {args.mode} mode, shortest survival {worst:.1f} s.')
        if args.mode == 'buzz':
            print('  Try --ping next. If that holds far longer, the motors are '
                  'browning\n  out the radio: suspect the battery or supply '
                  'rail, not the firmware.')
        elif args.mode == 'ping':
            print('  Try --idle next. If that holds far longer, the drop is in '
                  'the write\n  path rather than the link itself.')
        else:
            print('  It drops with no traffic at all, so this is RF, firmware, '
                  'or a\n  supervision timeout - your writes are not the cause.')
    print()


def write_csv(path, rows):
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=['trial', 'started', 'mode',
                                           'motors', 'duty', 'connected',
                                           'connect_s', 'uptime_s', 'writes',
                                           'reason'])
        if new:
            w.writeheader()
        w.writerows(rows)
    print(f'  results appended to {path}\n')


def main():
    p = argparse.ArgumentParser(
        description='Measure how long the haptic belt stays connected over BLE.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--name', default=DEFAULT_NAME,
                   help=f'BLE advertised name (default: {DEFAULT_NAME})')
    p.add_argument('--address', default=None,
                   help='BLE MAC address of the belt to use (overrides --name)')
    p.add_argument('--scan', nargs='?', type=float, const=6.0, default=None,
                   metavar='SECONDS',
                   help='list every BLE device in range and exit (default 6 s)')

    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--ping', dest='mode', action='store_const', const='ping',
                      help='write a silent OFF every --interval (default)')
    mode.add_argument('--idle', dest='mode', action='store_const', const='idle',
                      help='hold the link with no traffic at all')
    mode.add_argument('--buzz', dest='mode', action='store_const', const='buzz',
                      help='pulse all 6 motors every --interval - worst-case '
                           'current draw')
    p.set_defaults(mode='ping')

    p.add_argument('--trials', type=int, default=1,
                   help='connect/hold cycles to run (default: 1)')
    p.add_argument('--max-hold', type=float, default=MAX_HOLD, metavar='SECONDS',
                   help=f'stop holding after this long and call it a pass; '
                        f'0 = hold until it drops (default: {MAX_HOLD:g})')
    p.add_argument('--interval', type=float, default=INTERVAL, metavar='SECONDS',
                   help=f'seconds between pings/buzzes (default: {INTERVAL:g})')
    p.add_argument('--buzz-ms', type=int, default=BUZZ_MS,
                   help=f'pulse length in --buzz mode (default: {BUZZ_MS})')
    p.add_argument('--motors', type=int, default=BELT_SIZE, metavar='N',
                   help=f'how many motors fire at once in --buzz mode '
                        f'(default: all {BELT_SIZE})')
    p.add_argument('--duty', type=int, default=100,
                   help='duty %% in --buzz mode (default: 100)')
    p.add_argument('--ramp', action='store_true',
                   help='buzz mode with 1, 2, ... N motors in turn, to find the '
                        'load at which the link breaks. Implies --buzz')
    p.add_argument('--ma-per-motor', type=float, default=80.0, metavar='MA',
                   help='per-motor current used to turn a --ramp threshold into '
                        'an estimated milliamp figure (default: 80)')
    p.add_argument('--settle', type=float, default=SETTLE, metavar='SECONDS',
                   help=f'pause between trials (default: {SETTLE:g})')
    p.add_argument('--csv', default=None, metavar='PATH',
                   help='append the per-trial results to this CSV')
    args = p.parse_args()

    if args.scan is not None:
        print_scan(args.scan, match=args.name)
        return 0

    target = args.address or f'name {args.name!r}'

    if args.ramp:
        args.mode = 'buzz'
        # One trial per motor count: 1 motor, 2 motors, ... up to --motors.
        loads = list(range(1, max(1, min(args.motors, BELT_SIZE)) + 1))
    else:
        loads = [min(args.motors, BELT_SIZE)] * args.trials

    limit = f'{args.max_hold:g} s' if args.max_hold else 'until it drops'
    detail = ('no traffic' if args.mode == 'idle' else
              f'every {args.interval:g} s'
              + (f', {args.buzz_ms} ms @ {args.duty}%'
                 if args.mode == 'buzz' else ''))
    print(f'\nBLE uptime test - mode: {args.mode} ({detail})')
    print(f'target: {target}   trials: {len(loads)}   hold: {limit}')
    if args.ramp:
        print(f'ramp: {loads[0]} to {loads[-1]} motors at once, '
              f'{args.max_hold:g} s each')
    if args.mode == 'buzz':
        print('note: the belt will vibrate throughout - take it off first.')

    rows = []
    try:
        for n, motors in enumerate(loads, start=1):
            if n > 1 and args.settle:
                time.sleep(args.settle)
            rows.append(trial(n, args, target, motors))
    except KeyboardInterrupt:
        print('\n[stopped by user]')

    if rows:
        summarize(rows, args)
        if args.csv:
            write_csv(args.csv, rows)
    return 0


if __name__ == '__main__':
    sys.exit(main())
