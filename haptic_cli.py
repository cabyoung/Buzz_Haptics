#!/usr/bin/env python3
"""
haptic_cli.py — manual trigger console for the navigation haptic encoding.

Every signal defined in haptics_nav.NavHaptics is exposed here so you can fire
them one at a time, out of sequence, without stepping through the scripted demo
in haptics_nav.py. Useful for bench-checking the belt, tuning pulse parameters,
and running perceptual A/B comparisons on a wearer.

The signal table is derived from NavHaptics itself — add a method there and it
shows up here.

Usage
-----
    python haptic_cli.py                          # interactive, BLE backend
    python haptic_cli.py --backend dry            # no hardware, print only
    python haptic_cli.py --backend serial --port COM4
    python haptic_cli.py --address AA:BB:CC:DD:EE:FF

    python haptic_cli.py --list                   # print signal table, exit
    python haptic_cli.py --play turn_left         # fire one signal, exit
    python haptic_cli.py --play start obstacle cleared arrive --delay 2

Interactive commands (type `?` for the same list at the prompt)
--------------------------------------------------------------
    <n> | <name>        fire that signal          (e.g. `5`, `turn_left`, `tl`)
    <name> x<k>         fire it k times           (e.g. `obstacle x3`)
    pre <name>          fire only the pre-warning
    pair <name>         pre-warning, gap, then the actual signal
    m <motor> [effect]  one motor                 (e.g. `m 4 long_buzz`)
    g <m,m,..> [effect] several motors together   (e.g. `g 3,4,5 soft_bump`)
    sweep <l|r> [eff]   directional sweep from center
    all [effect]        every motor at once
    p <motor> <ms> <duty>   raw PULSE, BLE only   (e.g. `p 4 300 70`)
    scan                walk motors 0..8 one at a time
    off                 stop every motor
    freq <hz>           set the PWM carrier, BLE only
    gap [s]             show / set the inter-motor sweep gap
    rest [s]            show / set the pause between repeats
    effects             list effect names
    repeat | <enter>    fire the last thing again
    ? | help | list     show commands / signal table
    q | quit            stop motors and exit
"""

import argparse
import sys
import time

from buzzHaptics.Buzz_Haptics.haptics_nav import (
    NavHaptics,
    C_GRP,
    SOFT_BUMP,
    STRONG_CLICK,
    SOFT_BUZZ,
    LONG_BUZZ,
    EFFECT_PULSE,
)

BELT_SIZE = 9

# Effect names accepted anywhere an [effect] argument is taken. Short aliases
# exist because these get typed a lot during a fitting session.
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

# (key, aliases, method, pre-warning method or None, description)
# Mirrors the docstring table in haptics_nav.py.
SIGNALS = [
    ('start',             ('s',),        'start',             'pre_start',
     'center 3-4-5, STRONG_CLICK'),
    ('arrive',            ('a',),        'arrive',            'pre_arrive',
     'center 3-4-5, STRONG_CLICK'),
    ('obstacle',          ('o',),        'obstacle',          'pre_obstacle',
     'center 3-4-5, LONG_BUZZ'),
    ('cleared',           ('c',),        'cleared',           'pre_cleared',
     'center 3-4-5, STRONG_CLICK x2'),
    ('turn_left',         ('tl',),       'turn_left',         None,
     'sweep 3>2>1>0, STRONG_CLICK'),
    ('turn_right',        ('tr',),       'turn_right',        None,
     'sweep 5>6>7>8, STRONG_CLICK'),
    ('hard_turn_left',    ('htl', 'hl'), 'hard_turn_left',    'pre_hard_turn_left',
     'sweep 3>2>1>0, STRONG_CLICK'),
    ('hard_turn_right',   ('htr', 'hr'), 'hard_turn_right',   'pre_hard_turn_right',
     'sweep 5>6>7>8, STRONG_CLICK'),
    ('slight_turn_left',  ('stl', 'sl'), 'slight_turn_left',  'pre_slight_turn_left',
     'sweep 3>2>1>0, SOFT_BUMP'),
    ('slight_turn_right', ('str', 'sr'), 'slight_turn_right', 'pre_slight_turn_right',
     'sweep 5>6>7>8, SOFT_BUMP'),
]

PRE_DESC = {
    'pre_start':             'center SOFT_BUMP',
    'pre_arrive':            'center SOFT_BUMP',
    'pre_obstacle':          'center SOFT_BUZZ',
    'pre_cleared':           'center SOFT_BUMP x2',
    'pre_hard_turn_left':    'sweep 3>2>1>0, SOFT_BUMP',
    'pre_hard_turn_right':   'sweep 5>6>7>8, SOFT_BUMP',
    'pre_slight_turn_left':  'motor 3, SOFT_BUMP',
    'pre_slight_turn_right': 'motor 5, SOFT_BUMP',
}

# name -> signal tuple, including every alias
_LOOKUP = {}
for _sig in SIGNALS:
    _LOOKUP[_sig[0]] = _sig
    for _alias in _sig[1]:
        _LOOKUP[_alias] = _sig

PRE_GAP = 0.55  # matches the condition-2 gap in haptics_nav.py


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
        """Parse `4` or `3,4,5` into a validated motor list, or None."""
        try:
            motors = [int(p) for p in token.replace(' ', '').split(',') if p != '']
        except ValueError:
            self._err(f'bad motor list: {token}')
            return None
        if not motors:
            self._err('no motors given')
            return None
        bad = [m for m in motors if not 0 <= m < BELT_SIZE]
        if bad:
            self._err(f'motor out of range 0-{BELT_SIZE - 1}: {bad}')
            return None
        return motors

    # ── signals ───────────────────────────────────────────────────────────────

    def play_signal(self, sig, times=1, variant='main'):
        """variant: 'main' | 'pre' | 'pair'."""
        key, _aliases, method, pre_method, _desc = sig

        if variant in ('pre', 'pair') and pre_method is None:
            self._err(f'{key} has no pre-warning')
            if variant == 'pre':
                return
            variant = 'main'

        for i in range(times):
            if i:
                time.sleep(self.rest)
            if variant == 'pair':
                self._echo(f'pre {key}  [{PRE_DESC.get(pre_method, "")}]')
                getattr(self.nav, pre_method)()
                time.sleep(PRE_GAP)
                self._echo(f'>>> {key}')
                getattr(self.nav, method)()
            elif variant == 'pre':
                self._echo(f'pre {key}  [{PRE_DESC.get(pre_method, "")}]')
                getattr(self.nav, pre_method)()
            else:
                suffix = f'  ({i + 1}/{times})' if times > 1 else ''
                self._echo(f'>>> {key}{suffix}')
                getattr(self.nav, method)()

    # ── raw actuation ─────────────────────────────────────────────────────────

    def cmd_motor(self, args):
        if not args:
            self._err('usage: m <motor> [effect]')
            return
        motors = self._motors(args[0])
        effect = self._effect(args[1] if len(args) > 1 else None)
        if motors is None or effect is None:
            return
        self._echo(f'motors {motors} @ {EFFECT_NAMES.get(effect, effect)}')
        self._fire(motors, effect)

    def cmd_sweep(self, args):
        if not args:
            self._err('usage: sweep <l|r> [effect]')
            return
        side = args[0].lower()
        if side in ('l', 'left'):
            motors = [3, 2, 1, 0]
        elif side in ('r', 'right'):
            motors = [5, 6, 7, 8]
        else:
            self._err('direction must be l or r')
            return
        effect = self._effect(args[1] if len(args) > 1 else None)
        if effect is None:
            return
        self._echo(f'sweep {motors} @ {EFFECT_NAMES.get(effect, effect)}')
        self.nav._sweep(motors, effect)

    def cmd_all(self, args):
        effect = self._effect(args[0] if args else None)
        if effect is None:
            return
        self._echo(f'all 9 motors @ {EFFECT_NAMES.get(effect, effect)}')
        self._fire(range(BELT_SIZE), effect)

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
        if not 0 <= motor < BELT_SIZE:
            self._err(f'motor out of range 0-{BELT_SIZE - 1}')
            return
        self._echo(f'pulse motor {motor}  {on_ms} ms @ {duty}%')
        self.nav._link.pulse(motor, on_ms, duty)
        # The device times the pulse itself; wait it out so the prompt does not
        # come back before the wearer has felt anything.
        time.sleep(on_ms / 1000.0)

    def cmd_scan(self, args):
        effect = self._effect(args[0] if args else None)
        if effect is None:
            return
        self._echo(f'scan 0..8 @ {EFFECT_NAMES.get(effect, effect)}')
        for m in range(BELT_SIZE):
            print(f'    motor {m}')
            self._fire([m], effect)
            time.sleep(0.25)

    def cmd_off(self, _args):
        if self.backend == 'ble':
            self.nav._link.off()
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
            self._echo(f'sweep gap = {self.nav._gap:.3f} s')
            return
        try:
            self.nav._gap = max(0.0, float(args[0]))
        except ValueError:
            self._err('gap must be a number of seconds')
            return
        self._echo(f'sweep gap = {self.nav._gap:.3f} s')

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

    def cmd_list(self, _args):
        print()
        print('  #   signal               aliases     pattern                       pre-warning')
        print('  ' + '-' * 92)
        for n, sig in enumerate(SIGNALS, start=1):
            key, aliases, _m, pre, desc = sig
            pre_txt = PRE_DESC.get(pre, '-') if pre else '-'
            print(f'  {n:<3} {key:<20} {" ".join(aliases):<11} {desc:<29} {pre_txt}')
        print()
        print('  `pre <name>` fires only the warning, `pair <name>` fires warning + signal.')
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
            'm': self.cmd_motor, 'motor': self.cmd_motor,
            'g': self.cmd_motor, 'group': self.cmd_motor,
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
                           'gap', 'rest'):
                self.last = line
            return True

        if cmd == 'repeat':
            if self.last:
                return self.handle(self.last)
            self._err('nothing to repeat yet')
            return True

        # pre <name> / pair <name>
        variant = 'main'
        if cmd in ('pre', 'pair'):
            variant = cmd
            if not args:
                self._err(f'usage: {cmd} <signal>')
                return True
            cmd, args = args[0].lower(), args[1:]

        # trailing x<k> repeat count
        times = 1
        if args and args[0].lower().startswith('x') and args[0][1:].isdigit():
            times = max(1, int(args[0][1:]))

        # numeric index into the signal table
        if cmd.isdigit():
            idx = int(cmd)
            if not 1 <= idx <= len(SIGNALS):
                self._err(f'signal number must be 1-{len(SIGNALS)}')
                return True
            sig = SIGNALS[idx - 1]
        elif cmd in _LOOKUP:
            sig = _LOOKUP[cmd]
        else:
            self._err(f'unknown command: {cmd}  (type ? for help)')
            return True

        self.play_signal(sig, times=times, variant=variant)
        self.last = line
        return True


# ── entry point ───────────────────────────────────────────────────────────────

def build_nav(args):
    """Connect the requested backend, falling back to dry on failure."""
    backend = 'dry' if args.dry_run else args.backend
    try:
        nav = NavHaptics(backend=backend, port=args.port,
                         name=args.name, address=args.address)
        return nav, backend
    except Exception as exc:
        if backend == 'dry':
            raise
        print(f'\n! {backend} backend unavailable ({exc})')
        print('  falling back to dry run — commands will print but not actuate\n')
        return NavHaptics(backend='dry'), 'dry'


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
    p.add_argument('--list', action='store_true', dest='list_only',
                   help='print the signal table and exit')
    p.add_argument('--play', nargs='+', metavar='SIGNAL',
                   help='fire these signals in order, then exit '
                        '(prefix with pre: or pair: for the warning variants)')
    p.add_argument('--delay', type=float, default=1.5,
                   help='seconds between --play signals (default: 1.5)')
    args = p.parse_args()

    if args.list_only:
        HapticConsole.cmd_list(None, None)
        return 0

    nav, backend = build_nav(args)
    console = HapticConsole(nav, backend)

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
