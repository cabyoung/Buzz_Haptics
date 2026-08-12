"""
haptics_nav.py - transport + visualizer for the navigation haptic encoding.

The encoding itself lives in nav_signals.py, which is standalone and has no
imports beyond the standard library. This file is only the part that cannot be
handed to someone else: our BLE/serial transports, the Tk visualizer, and the
scripted route demo. If you want the signals, take nav_signals.py.

Belt layout: 6 actuators in a single horizontal row.
    User-facing motor labels: 1 2 3 4 5 6
    Internal belt indices   : 0 1 2 3 4 5

Signals (see nav_signals.py for the full timing table)
-----------------------------------------------------
Condition 1   obstacle, turn_left, turn_right, arrive
Condition 2   obstacle_left, obstacle_right, turn_left, turn_right, arrive
              - each announced by a warning, then a `timer` gap, then the body

    nav = NavHaptics(backend='ble')
    nav.play('turn_left', condition=1)
    nav.play('obstacle_left', condition=2)
    nav.warning('arrive')            # just the warning segment

Backends
--------
ble     : the nRF5340 haptic device in src/ (BLE peripheral, binary protocol).
          Pulses are one-shot PULSE commands the device times itself.
serial  : the legacy Arduino + DRV2605 belt (ASCII "motor:effect" @ 9600).
          On-times are approximated by the nearest library waveform.
dry     : no hardware, GUI only.

Usage
-----
python haptics_nav.py                       # BLE (default), condition 1
python haptics_nav.py --backend ble --c 2   # condition 2 walkthrough
python haptics_nav.py --backend dry         # GUI only, no hardware
python haptics_nav.py --c 2 --timer 3       # 3 s between warning and signal
python haptics_nav.py --backend serial --port COM4
python haptics_nav.py --address AA:BB:CC:DD:EE:FF
"""

import argparse
import time
import tkinter as tk
import threading
from dataclasses import replace

try:
    import nav_signals as sig                                  # run as a script
except ImportError:
    from buzzHaptics.Buzz_Haptics import nav_signals as sig    # run as a package

# Re-exported so callers can import everything from one place.
Params         = sig.Params
DEFAULT_PARAMS = sig.DEFAULT_PARAMS
C1_SIGNALS     = sig.C1_SIGNALS
C2_SIGNALS     = sig.C2_SIGNALS
BELT_SIZE      = sig.BELT_SIZE
BELT_ALL       = list(sig.ALL_MOTORS)
GRADIENT_L     = list(sig.GRADIENT_LEFT)
GRADIENT_R     = list(sig.GRADIENT_RIGHT)

PORT = "COM3"
BAUD = 9600

BLE_NAME = "HapticMotor"

# Display grouping for the visualizer only - no signal actuates by group. The
# encoding fires either the whole belt or one motor at a time.
L_GRP = [0, 1]
C_GRP = [2, 3]
R_GRP = [4, 5]

# ── bench-test effects (CLI only, not used by the study signals) ──────────────
# DRV2605 waveform-library IDs, kept so haptic_cli.py can drive individual
# motors with familiar effect names while fitting the belt.

SOFT_BUMP    = 22
STRONG_CLICK = 1
SOFT_BUZZ    = 47
LONG_BUZZ    = 58

EFFECT_INTENSITY = {
    SOFT_BUMP:    0.25,
    STRONG_CLICK: 0.60,
    SOFT_BUZZ:    0.45,
    LONG_BUZZ:    0.85,
}

# The DRV8837 H-bridges on the nRF5340 board have no waveform ROM, so each
# library effect is reproduced as a one-shot (on_ms, duty) pulse. Duty stays
# >= 40% because the ERMs will not spin up below that.
EFFECT_PULSE = {
    SOFT_BUMP:    (50,  45),
    STRONG_CLICK: (60,  100),
    SOFT_BUZZ:    (400, 55),
    LONG_BUZZ:    (600, 100),
}

# Gap between consecutive motors for the CLI's bench sweep. The serial backend
# needs a long gap for the Arduino round trip; BLE pulses are fire-and-forget.
SWEEP_GAP_SERIAL = 0.15
SWEEP_GAP_BLE    = 0.12
SWEEP_GAP_DRY    = 0.06

# Above this on-time the serial backend plays a buzz rather than a click.
SERIAL_CLICK_MAX_MS = 120


# ── haptics ───────────────────────────────────────────────────────────────────

class NavHaptics:
    """Transport + GUI callback for the encoding in nav_signals.py.

    backend: 'ble'    -> nRF5340 haptic device (src/), binary GATT protocol
             'serial' -> legacy Arduino + DRV2605 belt, ASCII "motor:effect"
             'dry'    -> no hardware

    params:  a nav_signals.Params; every duration in the encoding lives there.
    """

    def __init__(self, backend='ble', port=PORT, name=BLE_NAME, address=None,
                 on_motor=None, params=None):
        self._on_motor = on_motor
        self._backend  = backend
        self._link     = None
        self._ser      = None
        self.params    = params or sig.DEFAULT_PARAMS

        # Set by stop() to abandon a pattern that is mid-flight on another
        # thread (the GUI's close button, an experimenter's abort).
        self._stop_flag = threading.Event()

        if backend == 'ble':
            try:
                from haptic_ble import BleHapticLink          # run as a script
            except ImportError:
                from buzzHaptics.Buzz_Haptics.haptic_ble import BleHapticLink  # run as a package
            self._link = BleHapticLink(name=name, address=address).connect()
            self._gap = SWEEP_GAP_BLE
        elif backend == 'serial':
            import serial
            print(f"Connecting to {port}...")
            self._ser = serial.Serial(port, BAUD, timeout=2)
            time.sleep(2)
            ready = self._ser.read_all().decode(errors="ignore").strip()
            print(f"Arduino: {ready}")
            self._gap = SWEEP_GAP_SERIAL
        elif backend == 'dry':
            self._gap = SWEEP_GAP_DRY
        else:
            raise ValueError(f"unknown backend: {backend}")

    # ── transport ─────────────────────────────────────────────────────────────

    def _emit(self, motors, on_ms: int, duty: int):
        """Start `motors` for `on_ms` at `duty` % and return immediately.

        This is the one function nav_signals.play() needs; all timing is its
        responsibility, not ours."""
        if self._backend == 'ble':
            self._link.pulse_many(motors, on_ms, duty)
        elif self._backend == 'serial':
            # No on-time control here; pick the nearest library waveform.
            effect = STRONG_CLICK if on_ms <= SERIAL_CLICK_MAX_MS else LONG_BUZZ
            for m in motors:
                self._ser.write(f"{m}:{effect}\n".encode())
            self._ser.read_all()

        if self._on_motor:
            for m in motors:
                self._on_motor(m, duty / 100.0)

    # ── signals ───────────────────────────────────────────────────────────────

    def steps(self, signal, condition=1):
        """The pattern this instance would play, without playing it."""
        return sig.steps(signal, condition, self.params)

    def play(self, signal, condition=1):
        """Play one presentation of `signal`. Blocks for its full duration.
        Returns True if it ran to the end, False if stop() interrupted it."""
        return self._run(sig.steps(signal, condition, self.params))

    def warning(self, signal, condition=2):
        """Play only the warning segment of a condition-2 signal."""
        return self._run(sig.warning(signal, condition, self.params))

    def body(self, signal, condition=1):
        """Play only the signal body, skipping the warning and the gap."""
        return self._run(sig.body(signal, condition, self.params))

    def _run(self, pattern):
        self._stop_flag.clear()
        return sig.play(pattern, self._emit,
                        should_stop=self._stop_flag.is_set)

    def stop(self):
        """Abandon whatever is playing and silence the belt. Safe to call from
        another thread.

        The motors go quiet at once; the play() call itself returns at the next
        step boundary, which during the condition-2 timer can be a second or
        two later."""
        self._stop_flag.set()
        if self._backend == 'ble' and self._link is not None:
            try:
                self._link.off()
            except Exception:
                pass

    # Condition 1 -------------------------------------------------------------

    def obstacle(self):
        """All 6 actuators, 3 pulses in quick succession."""
        return self.play('obstacle', 1)

    def turn_left(self):
        """Motor 1 alone - the left edge of the belt."""
        return self.play('turn_left', 1)

    def turn_right(self):
        """Motor 6 alone - the right edge of the belt."""
        return self.play('turn_right', 1)

    def arrive(self):
        """One long buzz on the whole belt."""
        return self.play('arrive', 1)

    # Condition 2 -------------------------------------------------------------

    def c2_obstacle_left(self):
        """Buzz on motor 1, timer, then the left gradient twice."""
        return self.play('obstacle_left', 2)

    def c2_obstacle_right(self):
        """Buzz on motor 6, timer, then the right gradient twice."""
        return self.play('obstacle_right', 2)

    def c2_turn_left(self):
        """One buzz on motor 1, timer, then three more with a rest between."""
        return self.play('turn_left', 2)

    def c2_turn_right(self):
        """One buzz on motor 6, timer, then three more with a rest between."""
        return self.play('turn_right', 2)

    def c2_arrive(self):
        """Whole belt buzzes, timer, then one long buzz."""
        return self.play('arrive', 2)

    # ── bench actuation (CLI only; DRV2605 effect ids, not study signals) ─────

    def _fire(self, motors: list, effect: int):
        if self._backend == 'ble':
            on_ms, duty = EFFECT_PULSE.get(effect, (100, 80))
            self._link.pulse_many(motors, on_ms, duty)
        elif self._backend == 'serial':
            for m in motors:
                self._ser.write(f"{m}:{effect}\n".encode())

        time.sleep(self._gap)

        if self._backend == 'serial':
            self._ser.read_all()

        if self._on_motor:
            for m in motors:
                self._on_motor(m, EFFECT_INTENSITY.get(effect, 0.5))

    def _play(self, motor: int, effect: int):
        self._fire([motor], effect)

    def _sweep(self, motors: list, effect: int):
        for m in motors:
            self._play(m, effect)

    def _pulse(self, motors, on_ms: int, duty: int = 100):
        """One pulse, waited out. Used by the CLI's `buzz` command."""
        self._emit(list(motors), on_ms, duty)
        if on_ms > 0:
            time.sleep(on_ms / 1000.0)

    def close(self):
        self.stop()
        if self._link is not None:
            self._link.close()
        if self._ser is not None:
            self._ser.close()


# ── GUI ───────────────────────────────────────────────────────────────────────

class NavGUI:
    MAP_W, MAP_H = 360, 440
    BAR_W, BAR_H = 30, 200

    _PATH = [
        (255, 425),  # S  — bottom right
        (255, 340),  # hallway 1 (going up)
        (255, 250),
        (255, 237),  # hard left turn — clustered for sharp corner
        (244, 228),
        (228, 225),
        (190, 224),  # hallway 2 (going left)
        (130, 224),
        (90,  224),
        (78,  217),  # hard right turn — clustered for sharp corner
        (68,  204),
        (66,  188),
        (66,  110),  # hallway 3 (going up)
        (66,   25),  # ★ — top left
    ]

    # (cx, cy, which side of the wearer it sits on, given the travel direction)
    # Hallway 1 runs north and this one is east of the path -> on the right.
    # Hallway 2 runs west and this one is north of the path -> on the right.
    # Hallway 3 runs north and this one is west of the path -> on the left.
    _OBSTACLES = [
        (278, 325, 'on right'),
        (158, 207, 'on right'),
        (46,  118, 'on left'),
    ]

    _GROUPS = [
        ('LEFT',   L_GRP, '#42a5f5', '#1976d2'),
        ('CENTER', C_GRP, '#ffca28', '#ffa000'),
        ('RIGHT',  R_GRP, '#ef5350', '#c62828'),
    ]

    def __init__(self):
        self.root = tk.Tk()
        self.root.title('NavHaptics Visualizer')
        self.root.configure(bg='#0d0d1a')
        self.root.resizable(False, False)

        self._bar_val = [0.0] * BELT_SIZE
        self._bars    = [None] * BELT_SIZE
        self._robot_t = 0.0
        self._status  = tk.StringVar(value='Idle')
        self._running = True

        self._build()
        self._tick()

    # ── layout ────────────────────────────────────────────────────────────────

    def _build(self):
        outer = tk.Frame(self.root, bg='#0d0d1a')
        outer.pack(padx=14, pady=14)

        # left: map
        map_col = tk.Frame(outer, bg='#0d0d1a')
        map_col.pack(side=tk.LEFT, padx=(0, 18))

        self._map = tk.Canvas(map_col, width=self.MAP_W, height=self.MAP_H,
                               bg='#12122a', highlightthickness=0)
        self._map.pack()

        tk.Label(map_col, textvariable=self._status,
                  fg='#8888bb', bg='#0d0d1a',
                  font=('Helvetica', 12)).pack(pady=(6, 2))

        self._draw_map()
        self._update_entities()

        # right: bars
        bar_col = tk.Frame(outer, bg='#0d0d1a')
        bar_col.pack(side=tk.RIGHT, anchor=tk.N)

        tk.Label(bar_col, text=f'Haptic Belt  ({BELT_SIZE} motors)',
                  fg='white', bg='#0d0d1a',
                  font=('Helvetica', 12, 'bold')).pack(pady=(0, 10))

        groups_row = tk.Frame(bar_col, bg='#0d0d1a')
        groups_row.pack()

        for grp_name, motor_ids, hdr_clr, bar_clr in self._GROUPS:
            grp_frame = tk.Frame(groups_row, bg='#0d0d1a',
                                  highlightthickness=1,
                                  highlightbackground='#1e1e3e')
            grp_frame.pack(side=tk.LEFT, padx=6, pady=2, ipadx=4, ipady=4)

            tk.Label(grp_frame, text=grp_name, fg=hdr_clr, bg='#0d0d1a',
                      font=('Helvetica', 9, 'bold')).pack(pady=(0, 4))

            bars_row = tk.Frame(grp_frame, bg='#0d0d1a')
            bars_row.pack()

            for mid in motor_ids:
                col = tk.Frame(bars_row, bg='#0d0d1a')
                col.pack(side=tk.LEFT, padx=3)

                c = tk.Canvas(col, width=self.BAR_W, height=self.BAR_H,
                               bg='#1a1a38', highlightthickness=1,
                               highlightbackground='#2a2a50')
                c.pack()

                for pct in (25, 50, 75):
                    y = self.BAR_H - 2 - int(pct / 100 * (self.BAR_H - 4))
                    c.create_line(2, y, self.BAR_W - 2, y, fill='#2a2a50', width=1)

                fill_id = c.create_rectangle(
                    2, self.BAR_H - 2, self.BAR_W - 2, self.BAR_H - 2,
                    fill=bar_clr, outline=''
                )

                tk.Label(col, text=str(mid), fg=hdr_clr, bg='#0d0d1a',
                          font=('Helvetica', 8)).pack(pady=(2, 0))

                self._bars[mid] = (c, fill_id)

        # signal legend
        legend = tk.Frame(bar_col, bg='#0d0d1a')
        legend.pack(pady=(14, 0), anchor=tk.W)

        tk.Label(legend, text='Signals', fg='#aaa', bg='#0d0d1a',
                  font=('Helvetica', 9, 'bold')).grid(row=0, column=0, columnspan=2,
                                                       sticky='w', pady=(0, 4))

        p = sig.DEFAULT_PARAMS
        entries = [
            ('C1 obstacle',   f'all 6, {p.obstacle_pulses}x '
                              f'{p.obstacle_on_ms}/{p.obstacle_off_ms} ms'),
            ('C1 turn L / R', f'motor 1 or 6 alone, {p.turn_edge_ms} ms'),
            ('C1 arrive',     f'all 6, one {p.arrive_ms} ms buzz'),
            ('C2 obst L / R', f'buzz motor 1 or 6 -> away gradient '
                              f'x{p.obstacle_repeats}'),
            ('C2 turn L / R', f'1 buzz -> {p.turn_repeats} buzzes, motor 1 or 6'),
            ('C2 arrive',     f'all 6 buzz -> {p.arrive_long_ms} ms buzz'),
            ('C2 timer',      f'{p.warning_gap_ms} ms after every warning'),
        ]
        for r, (name, desc) in enumerate(entries, start=1):
            tk.Label(legend, text=name, fg='#ccc', bg='#0d0d1a',
                      font=('Helvetica', 8, 'bold'), anchor='w').grid(
                          row=r, column=0, sticky='w', padx=(0, 8))
            tk.Label(legend, text=desc, fg='#666', bg='#0d0d1a',
                      font=('Helvetica', 8), anchor='w').grid(
                          row=r, column=1, sticky='w')

    # ── map ───────────────────────────────────────────────────────────────────

    def _draw_map(self):
        pts  = self._PATH
        flat = [c for p in pts for c in p]
        self._map.create_line(*flat, fill='#0a0a20', width=22, smooth=True, capstyle='round')
        self._map.create_line(*flat, fill='#1e1e40', width=16, smooth=True, capstyle='round')
        self._map.create_line(*flat, fill='#28285a', width=12, smooth=True, capstyle='round')
        self._map.create_line(*flat, fill='#ffca28', width=1,  smooth=True, dash=(6, 8))

        sx, sy = pts[0]
        self._map.create_oval(sx-10, sy-10, sx+10, sy+10,
                               fill='#e65100', outline='#ffcc80', width=2)
        self._map.create_text(sx, sy, text='S', fill='white',
                               font=('Helvetica', 8, 'bold'))

        dx, dy = pts[-1]
        self._map.create_oval(dx-13, dy-13, dx+13, dy+13,
                               fill='#1b5e20', outline='#81c784', width=2)
        self._map.create_text(dx, dy, text='★', fill='#c8e6c9',
                               font=('Helvetica', 11))

        self._draw_obstacles()

    def _draw_obstacles(self):
        for cx, cy, label in self._OBSTACLES:
            r = 13
            self._map.create_rectangle(cx-r, cy-r, cx+r, cy+r,
                                        fill='#b71c1c', outline='#ff5252', width=2)
            self._map.create_text(cx, cy, text='!', fill='white',
                                   font=('Helvetica', 11, 'bold'))
            self._map.create_text(cx, cy + r + 9, text=label, fill='#ff8a80',
                                   font=('Helvetica', 7))

    def _path_pos(self, t: float):
        pts = self._PATH
        n   = len(pts) - 1
        seg = min(int(t * n), n - 1)
        f   = t * n - seg
        x0, y0 = pts[seg];  x1, y1 = pts[seg + 1]
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0)

    def _update_entities(self):
        rx, ry = self._path_pos(self._robot_t)
        px, py = self._path_pos(max(0.0, self._robot_t - 0.03))
        self._draw_dog(rx, ry)
        self._draw_person(px, py)

    def _draw_dog(self, x, y):
        self._map.delete('dog')
        self._map.create_oval(x-12, y-7,  x+12, y+7,
                               fill='#1565c0', outline='#90caf9', width=1.5, tags='dog')
        self._map.create_oval(x+8,  y-8,  x+19, y+3,
                               fill='#1976d2', outline='#90caf9', width=1.5, tags='dog')
        self._map.create_oval(x+12, y-13, x+18, y-7,
                               fill='#0d47a1', outline='#90caf9', width=1,   tags='dog')
        self._map.create_oval(x+14, y-6,  x+17, y-3,
                               fill='white',  outline='',         tags='dog')

    def _draw_person(self, x, y):
        self._map.delete('person')
        self._map.create_oval(x-5,  y-17, x+5,  y-7,
                               fill='#ffb74d', outline='#ffe0b2', width=1.5, tags='person')
        self._map.create_line(x,    y-7,  x,    y+9,  fill='#ffe0b2', width=2, tags='person')
        self._map.create_line(x-7,  y-2,  x+7,  y-2,  fill='#ffe0b2', width=2, tags='person')
        self._map.create_line(x,    y+9,  x-5,  y+19, fill='#ffe0b2', width=2, tags='person')
        self._map.create_line(x,    y+9,  x+5,  y+19, fill='#ffe0b2', width=2, tags='person')

    # ── public API ────────────────────────────────────────────────────────────

    def set_status(self, text: str):
        self.root.after(0, self._status.set, text)

    def flash_motor(self, motor: int, intensity: float):
        if 0 <= motor < BELT_SIZE:
            self._bar_val[motor] = max(self._bar_val[motor], intensity)

    def advance_path(self, target_t: float, steps: int = 20):
        start = self._robot_t

        def step(i):
            self._robot_t = min(start + (target_t - start) * i / steps, 1.0)
            self._update_entities()
            if i < steps:
                self.root.after(50, step, i + 1)

        self.root.after(0, step, 0)

    # ── tick ──────────────────────────────────────────────────────────────────

    def _tick(self):
        for i in range(BELT_SIZE):
            v = self._bar_val[i]
            self._bar_val[i] = max(0.0, v - 0.030) if v > 0.005 else 0.0
            c, fill_id = self._bars[i]
            h = int(self._bar_val[i] * (self.BAR_H - 4))
            c.coords(fill_id, 2, self.BAR_H - 2 - h, self.BAR_W - 2, self.BAR_H - 2)
        if self._running:
            self.root.after(50, self._tick)

    def run(self):
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self._running = False
        self.root.destroy()


# ── demo ──────────────────────────────────────────────────────────────────────

# The route: three obstacles, a hard left, a hard right, then arrival.
# (label, condition-1 signal, condition-2 signal, position along the path)
ROUTE = [
    ('Obstacle on the right', 'obstacle',   'obstacle_right', 0.10),
    ('Turn Left',             'turn_left',  'turn_left',      0.22),
    ('Obstacle on the right', 'obstacle',   'obstacle_right', 0.40),
    ('Turn Right',            'turn_right', 'turn_right',     0.58),
    ('Obstacle on the left',  'obstacle',   'obstacle_left',  0.75),
    ('Arrive',                'arrive',     'arrive',         1.00),
]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='NavHaptics demo')
    parser.add_argument('--backend', choices=['ble', 'serial', 'dry'], default='ble',
                         help='ble = nRF5340 haptic device, serial = Arduino belt, '
                              'dry = GUI only (default: ble)')
    parser.add_argument('--name', default=BLE_NAME,
                         help=f'BLE advertised name (default: {BLE_NAME})')
    parser.add_argument('--address', default=None,
                         help='BLE address (overrides --name)')
    parser.add_argument('--port', default=PORT, help='serial port (default: COM3)')
    parser.add_argument('--dry-run', action='store_true',
                         help='alias for --backend dry')
    parser.add_argument('--c', type=int, choices=[1, 2], default=1,
                         help='1 = no warning  2 = warning + timer + signal '
                              '(default: 1)')
    parser.add_argument('--timer', type=float,
                         default=sig.DEFAULT_PARAMS.warning_gap_ms / 1000.0,
                         help='condition 2: seconds between a warning and the '
                              'signal it announces (default: '
                              f'{sig.DEFAULT_PARAMS.warning_gap_ms / 1000.0:g})')
    args = parser.parse_args()

    backend = 'dry' if args.dry_run else args.backend
    params  = replace(sig.DEFAULT_PARAMS,
                      warning_gap_ms=int(round(args.timer * 1000)))

    gui = NavGUI()

    def demo():
        time.sleep(0.4)
        try:
            nav = NavHaptics(backend=backend, port=args.port,
                              name=args.name, address=args.address,
                              on_motor=lambda m, v: gui.flash_motor(m, v),
                              params=params)
        except Exception as exc:
            print(f'{backend} backend unavailable ({exc}) — switching to dry-run')
            gui.set_status('Hardware unavailable — dry run')
            nav = NavHaptics(backend='dry',
                              on_motor=lambda m, v: gui.flash_motor(m, v),
                              params=params)

        try:
            print(f'\n=== Condition {args.c} — '
                  + ('discrete signals, no warning' if args.c == 1
                     else f'warning + {args.timer:g} s timer + signal')
                  + ' ===\n')
            gui.set_status(f'Condition {args.c}')
            time.sleep(1.0)

            for label, c1_signal, c2_signal, t in ROUTE:
                signal = c1_signal if args.c == 1 else c2_signal
                secs = sig.duration_ms(nav.steps(signal, args.c)) / 1000.0
                print(f'  >>> {label:<22} {signal:<15} {secs:.2f} s')
                gui.set_status(label)
                gui.advance_path(t)
                nav.play(signal, args.c)
                time.sleep(1.5)

            gui.set_status('Arrived  ✓')
        except Exception as exc:
            # A mid-run disconnect must not leave motors latched on.
            print(f'[ERROR] run aborted: {exc}')
            gui.set_status(f'Aborted: {exc}')
        finally:
            nav.close()

    threading.Thread(target=demo, daemon=True).start()
    gui.run()
