"""
Edits: gui objects (1 as human-robot team), lengthen path for proper warning time, also we can combine
obstacle and the slight l/r signal

haptics_nav.py — navigation haptic encoding for a guide-dog robot.

Belt layout: 9 actuators in a single horizontal row.
  Motor IDs:  0   1   2   3   4   5   6   7   8
              ◄── LEFT ──► ◄─ CENTER ─► ◄── RIGHT ──►

Signals
-------
start             : 1 center buzz          (motors 3-4-5, STRONG_CLICK)
arrive            : 1 center buzz          (motors 3-4-5, STRONG_CLICK)
turn_left         : buzz sweep → left      (motors 3→2→1→0, STRONG_CLICK)
turn_right        : buzz sweep → right     (motors 5→6→7→8, STRONG_CLICK)
hard_turn_left    : strong sweep → left    (motors 3→2→1→0, STRONG_CLICK)
hard_turn_right   : strong sweep → right   (motors 5→6→7→8, STRONG_CLICK)
slight_turn_left  : soft sweep → left      (motors 3→2→1→0, SOFT_BUMP)
slight_turn_right : soft sweep → right     (motors 5→6→7→8, SOFT_BUMP)
obstacle          : 1 long center buzz     (motors 3-4-5, LONG_BUZZ)
cleared           : 2 strong center clicks (motors 3-4-5, STRONG_CLICK × 2)

Pre-warning patterns (condition 2 only — same motors, lower intensity):
  pre start/arrive  : center SOFT_BUMP
  pre hard turn L/R : sweep SOFT_BUMP   → actual sweep STRONG_CLICK
  pre slight turn   : single motor SOFT_BUMP → actual sweep SOFT_BUMP
  pre obstacle      : center SOFT_BUZZ  → actual center LONG_BUZZ
  pre cleared       : center SOFT_BUMP × 2 → actual center STRONG_CLICK × 2

Backends
--------
ble     : the nRF5340 haptic device in src/ (BLE peripheral, binary protocol).
          Effects become one-shot PULSE commands the device times itself.
serial  : the legacy Arduino + DRV2605 belt (ASCII "motor:effect" @ 9600).
dry     : no hardware, GUI only.

Usage
-----
python haptics_nav.py                       # BLE (default)
python haptics_nav.py --backend dry         # GUI only, no hardware
python haptics_nav.py --backend serial --port COM4
python haptics_nav.py --address AA:BB:CC:DD:EE:FF
"""

import argparse
import time
import tkinter as tk
import threading

PORT = "COM3"
BAUD = 9600

BLE_NAME = "HapticMotor"

L_GRP = [0, 1, 2]
C_GRP = [3, 4, 5]
R_GRP = [6, 7, 8]

SOFT_BUMP    = 22
STRONG_CLICK = 1
SOFT_BUZZ    = 47   # sustained but softer than LONG_BUZZ; used as pre-obstacle warning
LONG_BUZZ    = 58

EFFECT_INTENSITY = {
    SOFT_BUMP:    0.25,
    STRONG_CLICK: 0.60,
    SOFT_BUZZ:    0.45,
    LONG_BUZZ:    0.85,
}

# DRV2605 waveform IDs -> (on_ms, duty %) for the BLE backend. The DRV8837
# H-bridges on the nRF5340 board have no waveform ROM, so each library effect
# is reproduced as a one-shot pulse: clicks are short and sharp, buzzes are
# sustained. Duty stays >= 40% because the ERMs will not spin up below that.
EFFECT_PULSE = {
    SOFT_BUMP:    (50,  45),
    STRONG_CLICK: (60,  100),
    SOFT_BUZZ:    (400, 55),
    LONG_BUZZ:    (600, 100),
}

# Gap between consecutive motors in a sweep. The serial backend needs a long
# gap for the Arduino round trip; BLE pulses are fire-and-forget so the gap is
# purely perceptual (must stay above ~80ms to read as directional motion).
SWEEP_GAP_SERIAL = 0.15
SWEEP_GAP_BLE    = 0.12
SWEEP_GAP_DRY    = 0.06


# ── haptics ───────────────────────────────────────────────────────────────────

class NavHaptics:
    """Transport + GUI callback. All navigation signals in one encoding.

    backend: 'ble'    -> nRF5340 haptic device (src/), binary GATT protocol
             'serial' -> legacy Arduino + DRV2605 belt, ASCII "motor:effect"
             'dry'    -> no hardware
    """

    def __init__(self, backend='ble', port=PORT, name=BLE_NAME, address=None,
                 on_motor=None):
        self._on_motor = on_motor
        self._backend  = backend
        self._link     = None
        self._ser      = None

        if backend == 'ble':
            from buzzHaptics.Buzz_Haptics.haptic_ble import BleHapticLink
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

    def _fire(self, motors: list, effect: int):
        """Actuate every motor in `motors` with `effect`, then hold for one
        sweep gap. The GUI is flashed regardless of backend."""
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

    def _play_center(self, effect: int):
        """Fire motors 3-4-5 simultaneously."""
        self._fire(C_GRP, effect)

    def _sweep(self, motors: list, effect: int):
        for m in motors:
            self._play(m, effect)

    def _pause(self, s: float):
        time.sleep(s)

    def close(self):
        if self._link is not None:
            self._link.close()
        if self._ser is not None:
            self._ser.close()

    # ── signals ───────────────────────────────────────────────────────────────

    def start(self):
        """1 center buzz."""
        self._play_center(STRONG_CLICK)

    def arrive(self):
        """1 center buzz."""
        self._play_center(STRONG_CLICK)

    def obstacle(self):
        """1 long center buzz."""
        self._play_center(LONG_BUZZ)

    def cleared(self):
        """2 strong center clicks."""
        self._play_center(STRONG_CLICK)
        self._pause(0.35)
        self._play_center(STRONG_CLICK)

    def turn_left(self):
        """Normal buzz sweep toward left."""
        self._sweep([3, 2, 1, 0], STRONG_CLICK)

    def turn_right(self):
        """Normal buzz sweep toward right."""
        self._sweep([5, 6, 7, 8], STRONG_CLICK)

    def hard_turn_left(self):
        """Strong sweep toward left."""
        self._sweep([3, 2, 1, 0], STRONG_CLICK)

    def hard_turn_right(self):
        """Strong sweep toward right."""
        self._sweep([5, 6, 7, 8], STRONG_CLICK)

    def slight_turn_left(self):
        """Soft buzz sweep toward left."""
        self._sweep([3, 2, 1, 0], SOFT_BUMP)

    def slight_turn_right(self):
        """Soft buzz sweep toward right."""
        self._sweep([5, 6, 7, 8], SOFT_BUMP)

    # ── pre-warning signals (same pattern, lower intensity) ───────────────────

    def pre_start(self):
        self._play_center(SOFT_BUMP)

    def pre_arrive(self):
        self._play_center(SOFT_BUMP)

    def pre_obstacle(self):
        """Soft long buzz — softer than LONG_BUZZ actual."""
        self._play_center(SOFT_BUZZ)

    def pre_cleared(self):
        """2× soft center clicks — softer than 2× STRONG_CLICK actual."""
        self._play_center(SOFT_BUMP)
        self._pause(0.35)
        self._play_center(SOFT_BUMP)

    def pre_hard_turn_left(self):
        """Soft sweep — softer than STRONG_CLICK actual."""
        self._sweep([3, 2, 1, 0], SOFT_BUMP)

    def pre_hard_turn_right(self):
        self._sweep([5, 6, 7, 8], SOFT_BUMP)

    def pre_slight_turn_left(self):
        """Single center-left motor — softer than SOFT_BUMP sweep actual."""
        self._play(3, SOFT_BUMP)

    def pre_slight_turn_right(self):
        self._play(5, SOFT_BUMP)


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

    # (cx, cy, avoidance label)
    # hallway 1 obstacle is right of path → slight left
    # hallway 2 obstacle is above path   → slight right (curves below)
    # hallway 3 obstacle is left of path → slight right
    _OBSTACLES = [
        (278, 325, 'slight L'),
        (158, 207, 'slight R'),
        (46,  118, 'slight R'),
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

        self._bar_val = [0.0] * 9
        self._bars    = [None] * 9
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

        tk.Label(bar_col, text='Haptic Belt  (9 motors)',
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

        entries = [
            ('Start / Arrive',      '1× center  STRONG'),
            ('Turn L / R',          'sweep center→edge  STRONG'),
            ('Hard Turn L / R',     'sweep center→edge  INTENSE'),
            ('Slight Turn L / R',   'sweep center→edge  SOFT'),
            ('Obstacle',            '1× center  LONG'),
            ('Cleared',             '2× center  LONG'),
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
        if 0 <= motor < 9:
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
        for i in range(9):
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
                         help='1 = no pre-warning  2 = with pre-warning (default: 1)')
    args = parser.parse_args()

    backend = 'dry' if args.dry_run else args.backend

    gui = NavGUI()

    def demo():
        time.sleep(0.4)
        try:
            nav = NavHaptics(backend=backend, port=args.port,
                              name=args.name, address=args.address,
                              on_motor=lambda m, v: gui.flash_motor(m, v))
        except Exception as exc:
            print(f'{backend} backend unavailable ({exc}) — switching to dry-run')
            gui.set_status('Hardware unavailable — dry run')
            nav = NavHaptics(backend='dry',
                              on_motor=lambda m, v: gui.flash_motor(m, v))

        PRE_GAP = 0.55  # pause between pre-warning and actual signal

        try:
            if args.c == 1:
                # ── Condition 1: no pre-warnings ──────────────────────────────
                # Signals: Start, Turn L/R, Obstacle, Arrive
                print('\n=== Condition 1 — no pre-warning ===\n')
                gui.set_status('Condition 1')
                time.sleep(1.0)

                sequence = [
                    ('Start',     nav.start,      0.05),
                    ('Obstacle',  nav.obstacle,   0.10),
                    ('Turn Left', nav.turn_left,  0.22),
                    ('Obstacle',  nav.obstacle,   0.40),
                    ('Turn Right',nav.turn_right, 0.58),
                    ('Obstacle',  nav.obstacle,   0.75),
                    ('Arrive',    nav.arrive,     1.00),
                ]
                for label, fn, t in sequence:
                    print(f'  >>> {label}')
                    gui.set_status(label)
                    gui.advance_path(t)
                    fn()
                    time.sleep(1.5)

            else:
                # ── Condition 2: with pre-warnings ────────────────────────────
                # Signals: Start, Hard Turn L/R, Slight Turn L/R,
                #          Obstacle, Cleared, Arrive
                # Each signal is preceded by the same pattern at lower intensity.
                print('\n=== Condition 2 — with pre-warning ===\n')
                gui.set_status('Condition 2')
                time.sleep(1.0)

                # (label, pre_fn or None, actual_fn, path_t)
                sequence = [
                    ('Start',            nav.pre_start,             nav.start,             0.05),
                    ('Obstacle',         nav.pre_obstacle,           nav.obstacle,          0.08),
                    ('Slight Turn Left', None,                        nav.slight_turn_left,  0.11),
                    ('Cleared',          None,                        nav.cleared,           0.20),
                    ('Hard Turn Left',   nav.pre_hard_turn_left,     nav.hard_turn_left,    0.22),
                    ('Obstacle',         nav.pre_obstacle,           nav.obstacle,          0.48),
                    ('Slight Turn Left',None,                        nav.slight_turn_left, 0.50),
                    ('Cleared',          None,                        nav.cleared,           0.62),
                    ('Hard Turn Right',  nav.pre_hard_turn_right,    nav.hard_turn_right,   0.70),
                    ('Obstacle',         nav.pre_obstacle,           nav.obstacle,          0.90),
                    ('Slight Turn Right',None,                        nav.slight_turn_right, 0.95),
                    ('Cleared',          None,                        nav.cleared,           0.98),
                    ('Arrive',           nav.pre_arrive,             nav.arrive,            1.00),
                ]
                for label, pre_fn, fn, t in sequence:
                    gui.advance_path(t)
                    if pre_fn is not None:
                        print(f'  >>> ⚠ pre: {label}')
                        gui.set_status(f'⚠  {label}')
                        pre_fn()
                        time.sleep(PRE_GAP)
                    print(f'      → {label}')
                    gui.set_status(label)
                    fn()
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