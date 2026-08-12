#!/usr/bin/env python3
"""
haptic_cli.py — manual trigger console for the navigation haptic encoding.

Every signal defined in nav_signals.py is exposed here so you can fire them one
at a time, out of sequence, without stepping through the scripted demo in
haptics_nav.py. Useful for bench-checking the belt, tuning the timing
parameters, and running perceptual A/B comparisons on a wearer.

Signals are named the same everywhere. Condition 1 fires the bare signal;
condition 2 fires warning -> timer -> body.

Usage
-----
    python haptic_cli.py                          # interactive, BLE backend
    python haptic_cli.py --backend dry            # no hardware, print only
    python haptic_cli.py --backend serial --port COM4
    python haptic_cli.py --address AA:BB:CC:DD:EE:FF

    python haptic_cli.py --list                   # print signal table, exit
    python haptic_cli.py --steps                  # print every timeline, exit
    python haptic_cli.py --play turn_left         # fire one signal, exit
    python haptic_cli.py --play obstacle c2:turn_left c2:arrive --delay 2

Interactive commands (type `?` for the same list at the prompt)
--------------------------------------------------------------
    <n> | <name>        fire the condition-1 signal   (e.g. `2`, `turn_left`)
    <name> x<k>         fire it k times               (e.g. `obstacle x3`)
    c2 <name>           fire the condition-2 version  (warning, timer, body)
    warn <name>         fire only the condition-2 warning
    body <name>         fire only the body, skipping warning and timer
    steps <name> [1|2]  print the timeline, fire nothing
    timer [s]           show / set the warning->signal gap (condition 2)
    buzz                buzz every motor 1..6, one at a time
    buzz <m|all> [ms] [duty]
                        buzz one motor / a list / all six  (e.g. `buzz 4 500`,
                        `buzz 2,4,6`, `buzz all 500 60`)
    m <motor> [effect]  one motor                 (e.g. `m 4 long_buzz`)
    g <m,m,..> [effect] several motors together   (e.g. `g 3,4 soft_bump`)
    sweep <l|r> [eff]   directional sweep, all 6
    all [effect]        every motor at once
    p <motor> <ms> <duty>   raw PULSE, BLE only   (e.g. `p 4 300 70`)
    scan                walk motors 1..6 one at a time
    off | stop          stop every motor
    freq <hz>           set the PWM carrier, BLE only
    gap [s]             show / set the inter-motor bench-sweep gap
    rest [s]            show / set the pause between repeats and buzz steps
    effects             list effect names
    repeat | <enter>    fire the last thing again
    ? | help | list     show commands / signal table
    q | quit            stop motors and exit
"""

import argparse
import sys
import time
from dataclasses import replace

try:
    from haptics_nav import (                                  # run as a script
        NavHaptics,
        BELT_SIZE,
        BELT_ALL,
        GRADIENT_L,
        GRADIENT_R,
        SOFT_BUMP,
        STRONG_CLICK,
        SOFT_BUZZ,
        LONG_BUZZ,
        EFFECT_PULSE,
    )
    import nav_signals as sig
except ImportError:
    from buzzHaptics.Buzz_Haptics.haptics_nav import (          # run as a package
        NavHaptics,
        BELT_SIZE,
        BELT_ALL,
        GRADIENT_L,
        GRADIENT_R,
        SOFT_BUMP,
        STRONG_CLICK,
        SOFT_BUZZ,
        LONG_BUZZ,
        EFFECT_PULSE,
    )
    from buzzHaptics.Buzz_Haptics import nav_signals as sig

# Effect names accepted anywhere an [effect] argument is taken. These drive the
# bench commands (m / g / sweep / all / scan) only — the study signals carry
# their own timing, from nav_signals.Params.
EFFECTS = {
    'soft_bump':    SOFT_BUMP,
    'strong_click': STRONG_CLICK,
    'soft_buzz':    SOFT_BUZZ,
    'long_buzz':    LONG_BUZZ,
    'sb':           SOFT_BUMP,
    'sc':           STRONG_CLICK,
    'sz':           SOFT_BUZZ,
    'lb':           LONG_BUZZ,
}

EFFECT_NAMES = {
    SOFT_BUMP:    'soft_bump',
    STRONG_CLICK: 'strong_click',
    SOFT_BUZZ:    'soft_buzz',
    LONG_BUZZ:    'long_buzz',
}

DEFAULT_EFFECT = STRONG_CLICK

# Defaults for the `buzz` command, which drives motors with a direct
# (on_ms, duty) pulse rather than the effect table above.
BUZZ_MS   = 300
BUZZ_DUTY = 100

# (key, aliases) for every signal, in table order. A signal exists in a
# condition if nav_signals lists it there — obstacle is C1-only, and the two
# sided obstacles are C2-only.
SIGNAL_ALIASES = {
    'obstacle':       ('o', 'ob'),
    'obstacle_left':  ('ol',),
    'obstacle_right': ('or',),
    'turn_left':      ('tl',),
    'turn_right':     ('tr',),
    'arrive':         ('a', 'ar'),
}

SIGNALS = list(sig.C1_SIGNALS) + [s for s in sig.C2_SIGNALS
                                  if s not in sig.C1_SIGNALS]

# name -> canonical signal, including every alias
_LOOKUP = {}
for _name in SIGNALS:
    _LOOKUP[_name] = _name
    for _alias in SIGNAL_ALIASES.get(_name, ()):
        _LOOKUP[_alias] = _name


def conditions_for(signal):
    """Which conditions define this signal."""
    return tuple(c for c, names in ((1, sig.C1_SIGNALS), (2, sig.C2_SIGNALS))
                 if signal in names)


# ── console ───────────────────────────────────────────────────────────────────

class HapticConsole:
    """Dispatches typed commands onto a NavHaptics instance."""

    def __init__(self, nav: NavHaptics, backend: str):
        self.nav      = nav
        self.backend  = backend
        self.rest     = 0.35   # pause between repeats
        self.last     = None   # last command line, for bare <enter>

    # ── plumbing ──────────────────────────────────────────────────────────────

    def _fire(self, motors, effect):
        self.nav._fire(list(motors), effect)

    def _echo(self, text):
        print(f'  {text}')

    def _err(self, text):
        print(f'  ! {text}')

    def _effect(self, token, default=DEFAULT_EFFECT):
        """Resolve an effect name or raw DRV2605 id. Returns None if invalid."""
        if token is None:
            return default
        key = token.lower()
        if key in EFFECTS:
            return EFFECTS[key]
        if key.isdigit():
            return int(key)
        self._err(f'unknown effect: {token}  (try `effects`)')
        return None

    def _motors(self, token):
        """Parse `4` or `3,4,5` into a validated 1-based motor list, or None."""
        try:
            motors = [int(p) for p in token.replace(' ', '').split(',') if p != '']
        except ValueError:
            self._err(f'bad motor list: {token}')
            return None
        if not motors:
            self._err('no motors given')
            return None
        bad = [m for m in motors if not 1 <= m <= BELT_SIZE]
        if bad:
            self._err(f'motor out of range 1-{BELT_SIZE}: {bad}')
            return None
        return [m - 1 for m in motors]

    def _signal(self, token):
        """Resolve a name, alias or table index to a signal. None if invalid."""
        key = token.lower()
        if key.isdigit():
            idx = int(key)
            if not 1 <= idx <= len(SIGNALS):
                self._err(f'signal number must be 1-{len(SIGNALS)}')
                return None
            return SIGNALS[idx - 1]
        if key in _LOOKUP:
            return _LOOKUP[key]
        self._err(f'unknown signal: {token}  (type ? for help)')
        return None

    def _check_condition(self, signal, condition):
        """Report the nav_signals error rather than raising at the prompt."""
        if condition in conditions_for(signal):
            return True
        other = conditions_for(signal)
        self._err(f'{signal} is not a condition-{condition} signal '
                  f'(it exists in condition {" and ".join(map(str, other))})')
        if condition == 2 and signal == 'obstacle':
            self._err('condition 2 splits it: try obstacle_left / obstacle_right')
        if condition == 1 and signal.startswith('obstacle_'):
            self._err('condition 1 has no side: try obstacle')
        return False

    # ── signals ───────────────────────────────────────────────────────────────

    def play_signal(self, signal, condition=1, times=1, part='all'):
        """part: 'all' | 'warn' | 'body'."""
        if not self._check_condition(signal, condition):
            return

        # Go through NavHaptics rather than nav_signals.play() directly: it
        # clears the stop flag first, so a signal fired after `off` still runs.
        show, run = {
            'all':  (self.nav.steps, self.nav.play),
            'warn': (lambda s, c: sig.warning(s, c, self.nav.params),
                     self.nav.warning),
            'body': (lambda s, c: sig.body(s, c, self.nav.params),
                     self.nav.body),
        }[part]
        pattern = show(signal, condition)
        secs    = sig.duration_ms(pattern) / 1000.0

        for i in range(times):
            if i:
                time.sleep(self.rest)
            suffix = f'  ({i + 1}/{times})' if times > 1 else ''
            label  = signal if part == 'all' else f'{part} {signal}'
            self._echo(f'>>> c{condition} {label}  '
                       f'({len(sig.timeline(pattern))} pulses, {secs:.2f} s)'
                       f'{suffix}')
            run(signal, condition)

    def cmd_steps(self, args):
        if not args:
            self._err('usage: steps <signal> [1|2]')
            return
        signal = self._signal(args[0])
        if signal is None:
            return
        if len(args) > 1:
            try:
                conditions = [int(args[1])]
            except ValueError:
                self._err('condition must be 1 or 2')
                return
        else:
            conditions = list(conditions_for(signal))
        for condition in conditions:
            if not self._check_condition(signal, condition):
                continue
            print()
            print(sig.describe(signal, condition, self.nav.params))
        print()

    def cmd_timer(self, args):
        current = self.nav.params.warning_gap_ms / 1000.0
        if not args:
            self._echo(f'condition-2 timer = {current:g} s')
            return
        try:
            secs = max(0.0, float(args[0]))
        except ValueError:
            self._err('timer must be a number of seconds')
            return
        self.nav.params = replace(self.nav.params,
                                  warning_gap_ms=int(round(secs * 1000)))
        self._echo(f'condition-2 timer = {secs:g} s')

    # ── raw actuation ─────────────────────────────────────────────────────────

    def cmd_buzz(self, args):
        """Buzz motors individually with a direct (on_ms, duty) pulse.

            buzz                every motor, 1..6, one at a time
            buzz 4              just motor 4
            buzz 2,4,6          those three, still one at a time
            buzz all            all six together
            buzz 4 500          motor 4 for 500 ms
            buzz all 500 60     all six, 500 ms at 60%

        Unlike `m` / `scan` this bypasses the DRV2605 effect table, so the
        on-time and duty are exactly what you type on every backend.
        """
        target   = args[0].lower() if args else 'each'
        together = target in ('all', 'together')

        if together or target in ('each', 'e'):
            motors = list(BELT_ALL)
        else:
            motors = self._motors(target)
            if motors is None:
                return
        timing = args[1:]

        on_ms, duty = BUZZ_MS, BUZZ_DUTY
        try:
            if timing:
                on_ms = int(timing[0])
            if len(timing) > 1:
                duty = int(timing[1])
        except ValueError:
            self._err('usage: buzz [<motor>|all] [on_ms] [duty]')
            return
        if on_ms <= 0:
            self._err('on_ms must be positive')
            return
        if not 0 <= duty <= 100:
            self._err('duty must be 0-100')
            return

        if together:
            self._echo(f'buzz motors {[m + 1 for m in motors]} together  '
                       f'{on_ms} ms @ {duty}%')
            self.nav._pulse(motors, on_ms, duty)
            return

        self._echo(f'buzz {len(motors)} motor(s) one at a time  '
                   f'{on_ms} ms @ {duty}%  ({self.rest:g} s apart)')
        for i, m in enumerate(motors):
            if i:
                time.sleep(self.rest)
            print(f'    motor {m + 1}')
            self.nav._pulse([m], on_ms, duty)

    def cmd_motor(self, args):
        if not args:
            self._err('usage: m <motor> [effect]')
            return
        motors = self._motors(args[0])
        effect = self._effect(args[1] if len(args) > 1 else None)
        if motors is None or effect is None:
            return
        display_motors = [m + 1 for m in motors]
        self._echo(f'motors {display_motors} @ {EFFECT_NAMES.get(effect, effect)}')
        self._fire(motors, effect)

    def cmd_sweep(self, args):
        if not args:
            self._err('usage: sweep <l|r> [effect]')
            return
        side = args[0].lower()
        if side in ('l', 'left'):
            motors = list(GRADIENT_L)
        elif side in ('r', 'right'):
            motors = list(GRADIENT_R)
        else:
            self._err('direction must be l or r')
            return
        effect = self._effect(args[1] if len(args) > 1 else None)
        if effect is None:
            return
        display_motors = [m + 1 for m in motors]
        self._echo(f'sweep {display_motors} @ {EFFECT_NAMES.get(effect, effect)}')
        self.nav._sweep(motors, effect)

    def cmd_all(self, args):
        effect = self._effect(args[0] if args else None)
        if effect is None:
            return
        self._echo(f'all {BELT_SIZE} motors @ {EFFECT_NAMES.get(effect, effect)}')
        self._fire(BELT_ALL, effect)

    def cmd_pulse(self, args):
        if len(args) < 3:
            self._err('usage: p <motor> <on_ms> <duty>')
            return
        if self.backend != 'ble':
            self._err(f'raw pulse needs the ble backend (current: {self.backend})')
            return
        try:
            motor, on_ms, duty = int(args[0]), int(args[1]), int(args[2])
        except ValueError:
            self._err('motor, on_ms and duty must be integers')
            return
        if not 1 <= motor <= BELT_SIZE:
            self._err(f'motor out of range 1-{BELT_SIZE}')
            return
        self._echo(f'pulse motor {motor}  {on_ms} ms @ {duty}%')
        self.nav._link.pulse(motor - 1, on_ms, duty)
        # The device times the pulse itself; wait it out so the prompt does not
        # come back before the wearer has felt anything.
        time.sleep(on_ms / 1000.0)

    def cmd_scan(self, args):
        effect = self._effect(args[0] if args else None)
        if effect is None:
            return
        self._echo(f'scan 1..{BELT_SIZE} @ {EFFECT_NAMES.get(effect, effect)}')
        for m in range(1, BELT_SIZE + 1):
            print(f'    motor {m}')
            self._fire([m - 1], effect)
            time.sleep(0.25)

    def cmd_off(self, _args):
        self.nav.stop()
        if self.backend == 'ble':
            self._echo('all motors off')
        else:
            # The DRV2605 protocol is one-shot per effect; nothing to cancel.
            self._echo(f'no stop command on the {self.backend} backend (effects are one-shot)')

    def cmd_freq(self, args):
        if not args:
            self._err('usage: freq <hz>')
            return
        if self.backend != 'ble':
            self._err(f'freq needs the ble backend (current: {self.backend})')
            return
        try:
            hz = int(args[0])
        except ValueError:
            self._err('frequency must be an integer')
            return
        self.nav._link.set_frequency(hz)
        self._echo(f'PWM carrier -> {hz} Hz')

    # ── settings ──────────────────────────────────────────────────────────────

    def cmd_gap(self, args):
        if not args:
            self._echo(f'bench sweep gap = {self.nav._gap:.3f} s')
            return
        try:
            self.nav._gap = max(0.0, float(args[0]))
        except ValueError:
            self._err('gap must be a number of seconds')
            return
        self._echo(f'bench sweep gap = {self.nav._gap:.3f} s')

    def cmd_rest(self, args):
        if not args:
            self._echo(f'repeat rest = {self.rest:.3f} s')
            return
        try:
            self.rest = max(0.0, float(args[0]))
        except ValueError:
            self._err('rest must be a number of seconds')
            return
        self._echo(f'repeat rest = {self.rest:.3f} s')

    # ── info ──────────────────────────────────────────────────────────────────

    def cmd_effects(self, _args):
        print()
        print('  effect        id   pulse (on_ms, duty)   aliases')
        print('  ' + '-' * 54)
        for eff_id, name in EFFECT_NAMES.items():
            on_ms, duty = EFFECT_PULSE.get(eff_id, ('-', '-'))
            aliases = ' '.join(a for a, v in EFFECTS.items()
                               if v == eff_id and len(a) <= 2)
            print(f'  {name:<13} {eff_id:<4} {on_ms:>4} ms @ {duty:>3}%        {aliases}')
        print()
        print('  Bench commands only — the study signals carry their own timing.')
        print()

    def cmd_list(self, _args):
        params = self.nav.params
        print()
        print('  #   signal           aliases   in    C1              C2')
        print('  ' + '-' * 74)
        for n, signal in enumerate(SIGNALS, start=1):
            conds   = conditions_for(signal)
            aliases = ' '.join(SIGNAL_ALIASES.get(signal, ()))
            cells = []
            for c in (1, 2):
                if c in conds:
                    d = sig.duration_ms(sig.steps(signal, c, params)) / 1000.0
                    cells.append(f'{d:.2f} s')
                else:
                    cells.append('-')
            print(f'  {n:<3} {signal:<16} {aliases:<9} '
                  f'{",".join(map(str, conds)):<5} {cells[0]:<15} {cells[1]}')
        print()
        print(f'  C1: `<name>`.   C2: `c2 <name>` — warning, '
              f'{params.warning_gap_ms / 1000.0:g} s timer, then the body.')
        print('  `warn <name>` / `body <name>` fire the halves; `steps <name>` '
              'prints the timeline.')
        print()

    def cmd_help(self, _args):
        print(__doc__.split('Interactive commands')[1].split('---\n', 1)[1])
        self.cmd_list(None)

    # ── dispatch ──────────────────────────────────────────────────────────────

    def handle(self, line: str) -> bool:
        """Run one command line. Returns False when the user wants to quit."""
        line = line.strip()
        if not line:
            if self.last is None:
                return True
            line = self.last
            print(f'  (repeat) {line}')

        parts = line.split()
        cmd, args = parts[0].lower(), parts[1:]

        if cmd in ('q', 'quit', 'exit'):
            return False

        simple = {
            '?': self.cmd_help, 'h': self.cmd_help, 'help': self.cmd_help,
            'l': self.cmd_list, 'list': self.cmd_list, 'ls': self.cmd_list,
            'effects': self.cmd_effects, 'e': self.cmd_effects,
            'steps': self.cmd_steps, 'timeline': self.cmd_steps,
            'timer': self.cmd_timer,
            'm': self.cmd_motor, 'motor': self.cmd_motor,
            'g': self.cmd_motor, 'group': self.cmd_motor,
            'b': self.cmd_buzz, 'buzz': self.cmd_buzz,
            'sweep': self.cmd_sweep,
            'all': self.cmd_all,
            'p': self.cmd_pulse, 'pulse': self.cmd_pulse,
            'scan': self.cmd_scan, 'test': self.cmd_scan,
            'off': self.cmd_off, 'stop': self.cmd_off,
            'freq': self.cmd_freq,
            'gap': self.cmd_gap,
            'rest': self.cmd_rest,
        }

        if cmd in simple:
            simple[cmd](args)
            # Info-only commands are noise when repeated with <enter>.
            if cmd not in ('?', 'h', 'help', 'l', 'list', 'ls', 'effects', 'e',
                           'gap', 'rest', 'steps', 'timeline', 'timer'):
                self.last = line
            return True

        if cmd == 'repeat':
            if self.last:
                return self.handle(self.last)
            self._err('nothing to repeat yet')
            return True

        # c1 / c2 / warn / body <name>
        condition, part = 1, 'all'
        if cmd in ('c1', 'c2', 'warn', 'body'):
            if cmd in ('c1', 'c2'):
                condition = int(cmd[1])
            else:
                condition, part = 2, cmd
            if not args:
                self._err(f'usage: {cmd} <signal>')
                return True
            cmd, args = args[0].lower(), args[1:]

        # trailing x<k> repeat count
        times = 1
        if args and args[0].lower().startswith('x') and args[0][1:].isdigit():
            times = max(1, int(args[0][1:]))

        signal = self._signal(cmd)
        if signal is None:
            return True

        self.play_signal(signal, condition=condition, times=times, part=part)
        self.last = line
        return True


# ── entry point ───────────────────────────────────────────────────────────────

def build_nav(args, params):
    """Connect the requested backend, falling back to dry on failure."""
    backend = 'dry' if args.dry_run else args.backend
    try:
        nav = NavHaptics(backend=backend, port=args.port, name=args.name,
                         address=args.address, params=params)
        return nav, backend
    except Exception as exc:
        if backend == 'dry':
            raise
        print(f'\n! {backend} backend unavailable ({exc})')
        print('  falling back to dry run — commands will print but not actuate\n')
        return NavHaptics(backend='dry', params=params), 'dry'


def main():
    p = argparse.ArgumentParser(
        description='Manual trigger console for the navigation haptic encoding.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--backend', choices=['ble', 'serial', 'dry'], default='ble',
                   help='ble = nRF5340 device, serial = Arduino belt, dry = no hardware')
    p.add_argument('--name', default='HapticMotor', help='BLE advertised name')
    p.add_argument('--address', default=None, help='BLE address (overrides --name)')
    p.add_argument('--port', default='COM3', help='serial port (default: COM3)')
    p.add_argument('--dry-run', action='store_true', help='alias for --backend dry')
    p.add_argument('--timer', type=float,
                   default=sig.DEFAULT_PARAMS.warning_gap_ms / 1000.0,
                   help='condition 2: seconds between a warning and the signal '
                        'it announces (default: '
                        f'{sig.DEFAULT_PARAMS.warning_gap_ms / 1000.0:g})')
    p.add_argument('--list', action='store_true', dest='list_only',
                   help='print the signal table and exit')
    p.add_argument('--steps', action='store_true', dest='steps_only',
                   help='print every signal timeline and exit')
    p.add_argument('--play', nargs='+', metavar='SIGNAL',
                   help='fire these signals in order, then exit '
                        '(prefix with c2: warn: or body: for the variants)')
    p.add_argument('--delay', type=float, default=1.5,
                   help='seconds between --play signals (default: 1.5)')
    args = p.parse_args()

    params = replace(sig.DEFAULT_PARAMS,
                     warning_gap_ms=int(round(args.timer * 1000)))

    if args.steps_only:
        for condition, names in ((1, sig.C1_SIGNALS), (2, sig.C2_SIGNALS)):
            for name in names:
                print()
                print(sig.describe(name, condition, params))
        print()
        return 0

    nav, backend = build_nav(args, params)
    console = HapticConsole(nav, backend)

    if args.list_only:
        console.cmd_list(None)
        nav.close()
        return 0

    try:
        if args.play:
            for i, token in enumerate(args.play):
                if i:
                    time.sleep(args.delay)
                variant, _, name = token.rpartition(':')
                line = f'{variant} {name}'.strip() if variant else name
                console.handle(line)
            return 0

        print(f'\nHaptic trigger console — backend: {backend}')
        print('Type ? for commands, a number or name to fire a signal, q to quit.')
        console.cmd_list(None)

        while True:
            try:
                line = input('haptic> ')
            except KeyboardInterrupt:
                # Ctrl-C mid-signal should silence the belt, not kill the tool.
                print()
                console.cmd_off(None)
                continue
            except EOFError:
                print()
                break
            try:
                if not console.handle(line):
                    break
            except Exception as exc:
                print(f'  ! command failed: {exc}')
    finally:
        try:
            nav.close()
        except Exception:
            pass
        print('bye')

    return 0


if __name__ == '__main__':
    sys.exit(main())
