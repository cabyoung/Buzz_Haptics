"""
haptics_nav.py — three haptic encoding schemes for a navigation user study.

Belt layout: 9 actuators in a single horizontal row.
  Motor IDs:  0   1   2   3   4   5   6   7   8
              ◄── LEFT ──► ◄─ CENTER ─► ◄── RIGHT ──►

Encodings
---------
spatial   : which motor group fires encodes the cue; effect is constant.
intensity : center 3 motors (3,4,5) fire once; vibration strength = cue.
combined  : motor location AND pattern both encode the cue (redundant).

Usage
-----
python haptics_nav.py --encoding spatial
python haptics_nav.py --encoding intensity --dry-run
python haptics_nav.py --encoding combined        # default
"""

import argparse
import serial
import time
import tkinter as tk
import threading

PORT  = "COM3"
BAUD  = 9600

# Motor groups — belt runs left (0) to right (8)
L_GRP = [0, 1, 2]
C_GRP = [3, 4, 5]
R_GRP = [6, 7, 8]
ALL   = list(range(9))
MID   = 4       # exact-center motor (used by FrequencyHaptics)

SOFT_BUMP    = 22
STRONG_CLICK = 1
DOUBLE_CLICK = 10
TRIPLE_CLICK = 12
SHARP_CLICK  = 14
PULSE        = 47
LONG_BUZZ    = 58
ALERT        = 70

EFFECT_INTENSITY = {
    SOFT_BUMP:    0.25,
    STRONG_CLICK: 0.60,
    DOUBLE_CLICK: 0.50,
    TRIPLE_CLICK: 0.55,
    SHARP_CLICK:  0.70,
    PULSE:        0.45,
    LONG_BUZZ:    0.85,
    ALERT:        1.00,
}


# ── base class ────────────────────────────────────────────────────────

class NavHaptics:
    """Serial I/O + GUI callback. Subclasses define signal patterns."""

    def __init__(self, port=PORT, on_motor=None, dry_run=False):
        self._on_motor = on_motor
        self._dry_run  = dry_run
        if not dry_run:
            print(f"Connecting to {port}...")
            self._ser = serial.Serial(port, BAUD, timeout=2)
            time.sleep(2)
            ready = self._ser.read_all().decode(errors="ignore").strip()
            print(f"Arduino: {ready}")

    def _play(self, motor: int, effect: int):
        if not self._dry_run:
            self._ser.write(f"{motor}:{effect}\n".encode())
            time.sleep(0.15)
            self._ser.read_all()
        else:
            time.sleep(0.15)
        if self._on_motor:
            self._on_motor(motor, EFFECT_INTENSITY.get(effect, 0.5))

    def _play_all(self, effect: int):
        """Send all 9 motor commands back-to-back (no inter-command sleep) so they
        trigger as close to simultaneously as the serial link allows, then wait once."""
        if not self._dry_run:
            for m in ALL:
                self._ser.write(f"{m}:{effect}\n".encode())
            time.sleep(0.15)
            self._ser.read_all()
        else:
            time.sleep(0.15)
        if self._on_motor:
            for m in ALL:
                self._on_motor(m, EFFECT_INTENSITY.get(effect, 0.5))

    def _pause(self, s: float):
        time.sleep(s)

    def close(self):
        if not self._dry_run:
            self._ser.close()


# ── encoding 1: spatial ───────────────────────────────────────────────

class SpatialHaptics(NavHaptics):
    """
    SPATIAL — which motor group fires encodes the cue.
    Effect (STRONG_CLICK) is identical for all signals; only location varies.

    turn_left  → L group, inward-to-outward sweep (2→1→0) ×1
    turn_right → R group, inward-to-outward sweep (6→7→8) ×1
    obstacle   → C group, (3→4→5) ×1
    cleared    → all 9, STRONG_CLICK L→R
    starting   → full L→R wave (0→…→8)
    arriving   → edges-to-center converge (0,8 → 1,7 → … → 4)
    """

    def turn_left(self):
        for m in [2, 1, 0]:
            self._play(m, STRONG_CLICK)

    def turn_right(self):
        for m in [6, 7, 8]:
            self._play(m, STRONG_CLICK)

    def obstacle(self):
        for m in C_GRP:
            self._play(m, STRONG_CLICK)

    def cleared(self):
        for m in ALL:
            self._play(m, STRONG_CLICK)

    def starting(self):
        for m in ALL:
            self._play(m, STRONG_CLICK)

    def arriving(self):
        for m in [0, 8, 1, 7, 2, 6, 3, 5, 4]:
            self._play(m, STRONG_CLICK)


# ── encoding 2: intensity ─────────────────────────────────────────────

class IntensityHaptics(NavHaptics):
    """
    INTENSITY — center 3 motors (3, 4, 5) always fire simultaneously, exactly once.
    No spatial or frequency variation; only vibration strength differs per cue.

    starting   → SOFT_BUMP    (weakest)
    cleared    → PULSE        (low-medium)
    turn_left  → STRONG_CLICK (medium)
    turn_right → SHARP_CLICK  (medium-strong)
    arriving   → LONG_BUZZ    (strong)
    obstacle   → ALERT        (strongest)
    """

    def _m(self, effect):
        """Fire C_GRP (3,4,5) simultaneously — burst send, one wait."""
        if not self._dry_run:
            for m in C_GRP:
                self._ser.write(f"{m}:{effect}\n".encode())
            time.sleep(0.15)
            self._ser.read_all()
        else:
            time.sleep(0.15)
        if self._on_motor:
            for m in C_GRP:
                self._on_motor(m, EFFECT_INTENSITY.get(effect, 0.5))

    def turn_left(self):
        self._m(STRONG_CLICK)

    def turn_right(self):
        self._m(SHARP_CLICK)

    def obstacle(self):
        self._m(ALERT)

    def cleared(self):
        self._m(PULSE)

    def starting(self):
        self._m(SOFT_BUMP)

    def arriving(self):
        self._m(LONG_BUZZ)


# ── encoding 3: combined ──────────────────────────────────────────────

class CombinedHaptics(NavHaptics):
    """
    COMBINED — motor location AND temporal pattern both encode the cue.

    turn_left  → L group, sweep ×2  (left + count-2)
    turn_right → R group, sweep ×3  (right + count-3)
    obstacle   → C group, escalating soft→strong→buzz  (center + intensity)
    cleared    → all 9, soft spread
    starting   → full L→R wave
    arriving   → edges-to-center converge + long buzz
    """

    def turn_left(self):
        for _ in range(2):
            for m in [2, 1, 0]:
                self._play(m, SHARP_CLICK)
            self._pause(0.20)

    def turn_right(self):
        for _ in range(3):
            for m in [6, 7, 8]:
                self._play(m, SHARP_CLICK)
            self._pause(0.15)

    def obstacle(self):
        for m in C_GRP:
            self._play(m, SOFT_BUMP)
        self._pause(0.15)
        for m in C_GRP:
            self._play(m, STRONG_CLICK)
        self._pause(0.15)
        for m in C_GRP:
            self._play(m, LONG_BUZZ)

    def cleared(self):
        for m in ALL:
            self._play(m, SOFT_BUMP)

    def starting(self):
        for m in ALL:
            self._play(m, STRONG_CLICK)

    def arriving(self):
        for m in [0, 8, 1, 7, 2, 6, 3, 5, 4]:
            self._play(m, TRIPLE_CLICK)
        self._pause(0.30)
        for m in ALL:
            self._play(m, LONG_BUZZ)


ENCODINGS = {
    'spatial':   SpatialHaptics,
    'intensity': IntensityHaptics,
    'combined':  CombinedHaptics,
}

ENCODING_DESC = {
    'spatial':   'Spatial — motor location = cue',
    'intensity': 'Intensity — vibration strength = cue  (motors 3-4-5)',
    'combined':  'Combined — location + pattern = cue',
}


# ── GUI ───────────────────────────────────────────────────────────────

class NavGUI:
    MAP_W, MAP_H = 360, 440
    BAR_W, BAR_H = 30, 200

    _PATH = [
        (180, 420), (180, 360), (180, 295),
        (130, 240), (85,  195), (80,  145),
        (115, 90),  (165, 50),  (195, 25),
    ]

    # Group metadata (name, motor ids, header color, bar color)
    _GROUPS = [
        ('LEFT',   L_GRP, '#42a5f5', '#1976d2'),
        ('CENTER', C_GRP, '#ffca28', '#ffa000'),
        ('RIGHT',  R_GRP, '#ef5350', '#c62828'),
    ]

    def __init__(self, encoding_name: str = 'combined'):
        self.root = tk.Tk()
        self.root.title('NavHaptics Visualizer')
        self.root.configure(bg='#0d0d1a')
        self.root.resizable(False, False)

        self._bar_val = [0.0] * 9   # one entry per motor (indexed by motor ID)
        self._bars    = [None] * 9  # (canvas, fill_id) per motor
        self._robot_t = 0.0
        self._status  = tk.StringVar(value='Idle')
        self._running = True
        self._enc     = encoding_name

        self._build()
        self._tick()

    # ── layout ────────────────────────────────────────────────────────

    def _build(self):
        outer = tk.Frame(self.root, bg='#0d0d1a')
        outer.pack(padx=14, pady=14)

        # ── left: map ──
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

        # ── right: bars ──
        bar_col = tk.Frame(outer, bg='#0d0d1a')
        bar_col.pack(side=tk.RIGHT, anchor=tk.N)

        tk.Label(bar_col, text='Haptic Belt  (9 motors)',
                  fg='white', bg='#0d0d1a',
                  font=('Helvetica', 12, 'bold')).pack(pady=(0, 3))

        enc_clr = {'spatial': '#42a5f5', 'frequency': '#ffca28', 'combined': '#81c784'}
        tk.Label(bar_col,
                  text=ENCODING_DESC.get(self._enc, self._enc),
                  fg=enc_clr.get(self._enc, '#aaa'),
                  bg='#0d0d1a', font=('Helvetica', 8, 'italic'),
                  wraplength=280).pack(pady=(0, 10))

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

                # Tick marks
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

        # Belt position diagram (thin strip below bars)
        belt_canvas = tk.Canvas(bar_col, width=self.BAR_W * 9 + 60,
                                 height=24, bg='#0d0d1a', highlightthickness=0)
        belt_canvas.pack(pady=(8, 0))
        belt_canvas.create_text(self.BAR_W * 9 // 2 + 30, 12,
                                  text='◄  belt  ►', fill='#445',
                                  font=('Helvetica', 9))

    # ── map ───────────────────────────────────────────────────────────

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

    def _path_pos(self, t: float):
        pts = self._PATH
        n   = len(pts) - 1
        seg = min(int(t * n), n - 1)
        f   = t * n - seg
        x0, y0 = pts[seg];  x1, y1 = pts[seg + 1]
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0)

    def _update_entities(self):
        rx, ry = self._path_pos(self._robot_t)
        px, py = self._path_pos(max(0.0, self._robot_t - 0.10))
        self._draw_person(px, py)
        self._draw_dog(rx, ry)

    def _draw_dog(self, x, y):
        self._map.delete('dog')
        self._map.create_oval(x-12, y-7, x+12, y+7,
                               fill='#1565c0', outline='#90caf9', width=1.5, tags='dog')
        self._map.create_oval(x+8, y-8, x+19, y+3,
                               fill='#1976d2', outline='#90caf9', width=1.5, tags='dog')
        self._map.create_oval(x+12, y-13, x+18, y-7,
                               fill='#0d47a1', outline='#90caf9', width=1, tags='dog')
        self._map.create_oval(x+14, y-6, x+17, y-3,
                               fill='white', outline='', tags='dog')

    def _draw_person(self, x, y):
        self._map.delete('person')
        self._map.create_oval(x-5, y-17, x+5, y-7,
                               fill='#ffb74d', outline='#ffe0b2', width=1.5, tags='person')
        self._map.create_line(x, y-7, x, y+9,    fill='#ffe0b2', width=2, tags='person')
        self._map.create_line(x-7, y-2, x+7, y-2, fill='#ffe0b2', width=2, tags='person')
        self._map.create_line(x, y+9, x-5, y+19,  fill='#ffe0b2', width=2, tags='person')
        self._map.create_line(x, y+9, x+5, y+19,  fill='#ffe0b2', width=2, tags='person')

    # ── public API ────────────────────────────────────────────────────

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

    # ── tick ──────────────────────────────────────────────────────────

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


# ── demo ──────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='NavHaptics user-study demo')
    parser.add_argument('--encoding', choices=list(ENCODINGS), default='combined',
                         help='haptic encoding scheme (default: combined)')
    parser.add_argument('--dry-run', action='store_true',
                         help='skip serial connection (GUI-only)')
    args = parser.parse_args()

    gui = NavGUI(encoding_name=args.encoding)

    def demo():
        time.sleep(0.4)
        HapticsClass = ENCODINGS[args.encoding]
        print(f'\nEncoding: {ENCODING_DESC[args.encoding]}\n')
        try:
            nav = HapticsClass(on_motor=lambda m, v: gui.flash_motor(m, v),
                                dry_run=args.dry_run)
        except Exception as exc:
            print(f'Serial unavailable ({exc}) — switching to dry-run')
            nav = HapticsClass(on_motor=lambda m, v: gui.flash_motor(m, v), dry_run=True)

        sequence = [
            ('Starting',         nav.starting,  0.15),
            ('Turn Left',        nav.turn_left,  0.42),
            ('Turn Right',       nav.turn_right, 0.68),
            ('Obstacle ahead',   nav.obstacle,   0.68),
            ('Obstacle cleared', nav.cleared,    0.82),
            ('Arriving',         nav.arriving,   1.00),
        ]

        for label, fn, t in sequence:
            print(f'  >>> {label}')
            gui.set_status(label)
            gui.advance_path(t)
            fn()
            time.sleep(1.5)

        gui.set_status('Arrived  ✓')
        nav.close()

    threading.Thread(target=demo, daemon=True).start()
    gui.run()
