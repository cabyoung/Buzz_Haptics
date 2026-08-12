#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#: realcam.py
import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO
import math
import time
import logging
import os
import threading
from collections import deque
from datetime import datetime
from datetime import timedelta
import srt
from haptics_nav import NavHaptics, PORT
from haptic_ui_bridge import NavHapticsWithUI
import numpy as np
#from sensor_msgs.msg import PointCloud2
import struct
import requests
import socket
import subprocess
from queue import Queue, Empty
import pickle
import argparse
import re
import json

NAV_HAPTICS = None

# --- Camera Control Flag ---
camera_active = True

# --- Global Variables for Minimized SRT files ---
last_labels = []  # Store past labels for logging
dist_threshold = 0.2  # Distance threshold for object detection to avoid duplicates
class_depth_history = []  # Store past depth values for each class

# --- YOLO False Positive Filter ---
IGNORED_CLASSES = {"refrigerator", "elephant"}  # Classes known to be false positives in this environment
HAPTIC_TARGET_CLASS = "chair"  # Only emit haptic signals for this class
FOV_THRESHOLD = 30  # degrees - only detect objects within ±N degrees from straight ahead (center of image)


# --- Condition Presets ---
CONDITION_PRESETS = {
    1: {  # Condition 1: No warnings, fire AT waypoint (0 lookahead)
        "close_threshold": 5.0,
        "near_threshold": 8.0,
        "far_threshold": 10.0,
        "waypoint_lookahead": 0,  # Fire AT signal waypoint
        "description": "C1: No lookahead (signal fires AT waypoint)"
    },
    2: {  # Condition 2: With warnings, fire AHEAD of waypoint (2 lookahead)
        "close_threshold": 8.0,
        "near_threshold": 10.0,
        "far_threshold": 15.0,
        "waypoint_lookahead": 2,  # Fire 2 waypoints BEFORE signal
        "description": "C2: 2-waypoint lookahead (pre-warn user)"
    }
}

# --- Command Line Arguments ---
def parse_arguments():
    parser = argparse.ArgumentParser(description='Local camera with haptic feedback')
    parser.add_argument('--frame-skip', type=int, default=5,
                       help='Frame sampling interval: every Nth frame is queued (default: 5)')
    parser.add_argument('--condition', type=int, default=1, choices=[1, 2],
                       help='Haptic condition (1=no warnings/sensitive, 2=with warnings, default: 2)')
    parser.add_argument('--backend', type=str, default='ble', choices=['ble', 'dry', 'auto'],
                       help='Haptic backend: ble=Bluetooth device, dry=UI only, auto=try BLE then dry-run (default: ble)')
    parser.add_argument('--ble-address', type=str, default=None,
                       help='BLE device address (e.g., AA:BB:CC:DD:EE:FF). If not set, scans for HapticMotor')
    parser.add_argument('--pseudo', action='store_true',
                       help='Use pseudo/simulated camera data instead of real RealSense camera')
    return parser.parse_args()

ARGS = parse_arguments()
FRAME_SKIP = ARGS.frame_skip
CONDITION = ARGS.condition
BACKEND = ARGS.backend
BLE_ADDRESS = "FA:16:A4:5E:91:FB"
USE_PSEUDO_DATA = ARGS.pseudo

# Load preset thresholds based on condition
preset = CONDITION_PRESETS[CONDITION]
CLOSE_THRESHOLD = preset["close_threshold"]
NEAR_THRESHOLD = preset["near_threshold"]
FAR_THRESHOLD = preset["far_threshold"]

print("[CONFIG] {}".format(preset["description"]))
print("[CONFIG] Depth thresholds: close={:.2f}m, near={:.2f}m, far={:.2f}m".format(
    CLOSE_THRESHOLD, NEAR_THRESHOLD, FAR_THRESHOLD))

# Global PATH and related variables (needed by initialize_haptics -> check_path)
PATH = []
FULL_PATH = ""

# Cached arrival detection data (read once from FULL_PATH.txt at startup)
_arrival_first_waypoint = None
_arrival_last_waypoint = None
_arrival_expected_distance = None

# Waypoint progress tracking
_waypoint_list = []  # All waypoints from FULL_PATH.txt
_waypoint_data_with_headings = []  # Waypoints with headings: [(x, y, heading_deg), ...]
_last_reported_waypoint = -1  # Track which waypoint was last reported
_sim_start_time = None  # Track simulation start time

# Predictive turn detection (heading-based with smoothing/windowing)
_signaled_turn_indices = set()  # Track which turn waypoints already signaled
_sustained_turn_sequences = {}  # Track sustained turn sequences to avoid duplicate signals: {first_window_idx: turn_info}
if CONDITION == 1:
    SIGNAL_DISTANCE_THRESHOLD = 2.0  # meters - emit signal when within 2m of turn (increased from 1m to account for lookahead)
else:
    SIGNAL_DISTANCE_THRESHOLD = 4.0  # meters - emit signal when within 4m of turn (default)
WINDOW_SIZE = 8  # waypoints to consider for smoothed turn detection
HEADING_CHANGE_THRESHOLD = 35  # degrees - minimum net change to flag a real turn (filters staircase noise)
MIN_SUSTAINED_WINDOWS = 3  # Require turn to persist across 3+ consecutive windows (filters false positives)

# Cardinal direction ranges (degrees) - COMPASS-STYLE HEADING
# Compass convention: 0°=North(+y), 90°=East(+x), 180°=South(-y), -90°=West(-x)
# Each direction has ±45° range, expanded to cover all heading values
CARDINAL_RANGES = {
    'North': (-45, 45),           # 0° ± 45°
    'East': (45, 135),            # 90° ± 45°
    'South_Pos': (135, 180),      # 180° ± 45° (positive side)
    'South_Neg': (-180, -135),    # -180° ± 45° (negative side, wraps)
    'West': (-135, -45)           # -90° ± 45°
}

# Define check_path BEFORE calling initialize_haptics()
def check_path(filename="/home/halab2/buzz_shared/path_files/Path.txt", wait_timeout=60):
    global PATH, FULL_PATH
    FULL_PATH = ""

    print(f"[CHECK_PATH] ========== CHECK_PATH START ==========")
    print(f"[CHECK_PATH] Looking for file: {filename}")

    # Wait for file to exist (up to wait_timeout seconds)
    start_wait = time.time()
    while not os.path.exists(filename):
        elapsed = time.time() - start_wait
        if elapsed > wait_timeout:
            print(f"[CHECK_PATH] ❌ TIMEOUT: Path.txt not created within {wait_timeout}s")
            print(f"[CHECK_PATH] Robot will run without path until Path.txt appears")
            print(f"[CHECK_PATH] ========== CHECK_PATH END (timeout, PATH empty) ==========")
            return PATH
        remaining = wait_timeout - elapsed
        print(f"[CHECK_PATH] ⏳ Waiting for Path.txt... ({remaining:.0f}s remaining)")
        time.sleep(1)

    file_size = os.path.getsize(filename)
    print(f"[CHECK_PATH] ✓ File exists, size: {file_size} bytes")

    if file_size == 0:
        print(f"[CHECK_PATH] ⚠️  WARNING: Path.txt is EMPTY (0 bytes)")
        print(f"[CHECK_PATH] Waiting for robot_ui_v2.py to generate content...")
        time.sleep(2)
        if os.path.getsize(filename) == 0:
            print(f"[CHECK_PATH] ❌ Path.txt still empty after waiting")
            print(f"[CHECK_PATH] ========== CHECK_PATH END (loaded 0 commands) ==========")
            return PATH

    # Try to read the file (handle race condition where file gets deleted)
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
            print(f"[CHECK_PATH] Read {len(lines)} lines from file")
            PATH.clear()

            path_commands = 0
            for line_num, line in enumerate(lines):
                line = line.strip()
                if line.startswith("FULL PATH:"):
                    FULL_PATH = line[len("FULL PATH:"):].strip()
                    print(f"[CHECK_PATH] Found FULL_PATH at line {line_num}")
                    continue
                # ONLY add lines that start with "PATH:" (the actual commands)
                if line.startswith("PATH:"):
                    path_commands += 1
                    line = line[5:].strip()  # Remove "PATH:" prefix
                    PATH.append(line)

            print(f"[CHECK_PATH] ✓ Loaded {len(PATH)} command segments from Path.txt")
            print(f"[CHECK_PATH] Total PATH commands: {path_commands}")

            # Debug: Show first and last commands
            if len(PATH) > 0:
                print(f"[CHECK_PATH] FIRST command: {PATH[0]}")
                print(f"[CHECK_PATH] LAST command: {PATH[-1]}")

            # Debug: Show which commands are signals
            signal_cmds = [i for i, cmd in enumerate(PATH) if "turn" in cmd.lower() or "arrive" in cmd.lower()]
            print(f"[CHECK_PATH] Signal commands at indices: {signal_cmds}")
            if signal_cmds:
                for idx in signal_cmds[:5]:  # Show first 5 signal commands
                    print(f"[CHECK_PATH]   [{idx:3d}]: {PATH[idx]}")

            # CRITICAL: Check if arrive is at the end
            if len(PATH) > 0 and "arrive" in PATH[-1].lower():
                print(f"[CHECK_PATH] ✓✓✓ ARRIVE FOUND AT END (index {len(PATH)-1})")
            else:
                print(f"[CHECK_PATH] ❌❌❌ WARNING: ARRIVE NOT AT END OF PATH")

    except FileNotFoundError:
        print(f"[CHECK_PATH] ⚠️  File disappeared during read (race condition with robot_ui_v2.py)")
        print(f"[CHECK_PATH] Will retry next time PATH is checked")

    print(f"[CHECK_PATH] ========== CHECK_PATH END (loaded {len(PATH)} commands) ==========")
    return PATH

# Initialize haptics
print("[SCRIPT] local_cam_final_haptic.py started - about to initialize haptics...", flush=True)
#if not initialize_haptics():
#    raise RuntimeError("Failed to initialize haptic system")

# --- Frame Processing ---
FRAME_Q = Queue(maxsize=30)
processing_active = True

# --- Global Voice and Video
# filename array
VOICE = []
# video filename
VIDEO = ""
# --- Global SRT Files ---
BASE_OUTPUT_DIR = "/home/halab2/buzz_shared"
OUT_DIR = os.path.join(BASE_OUTPUT_DIR, "out_files")
IN_DIR = os.path.join(BASE_OUTPUT_DIR, "in_files")
AUDIO_DIR = os.path.join(BASE_OUTPUT_DIR, "audio_files")
VIDEO_DIR = os.path.join(BASE_OUTPUT_DIR, "video_files")
SCREENSHOT_DIR = os.path.join(BASE_OUTPUT_DIR, "screenshots")
TIMING_DIR = os.path.join(BASE_OUTPUT_DIR, "timing_data")
DETECTIONS_DIR = os.path.join(BASE_OUTPUT_DIR, "detections")
HAPTIC_DIR = os.path.join(BASE_OUTPUT_DIR, "haptic")

# State tracking for haptic signals - only emit when depth threshold is crossed
# Depth zones: None (no object), far (>1.5m), near (0.5-1.0m), close (<0.5m)
_previous_depth_zone = None  # Track which zone we're in (None = no obstacle)

# Path command state tracking - ensure each signal fires only once
_last_fired_path_command_index = -1  # Track which PATH command we last fired
_path_signal_state = {}  # Track fired state for each command index

# Robot progress tracking - for lookahead calculation
_robot_waypoint_progress = 0  # Current waypoint position (0-44 for 45-waypoint path)
_last_progress_update_time = 0  # Timestamp of last progress update

# Turn detection from odometry heading
_initial_heading = None  # Robot's heading at start
_previous_heading = None  # Robot's heading from last check
_last_turn_emission_time = 0  # Prevent turn spam (emit max once per 0.5s)
_turn_emit_cooldown = 0.5  # seconds between turn signal emissions

# Start signal detection from odometry movement
_initial_position = None  # Robot position at system start
_start_signal_fired = False  # Ensure start signal fires only once
_movement_threshold = 0.1  # meters - must move this far to trigger start

# Arrival signal detection from odometry distance
_arrive_signal_fired = False  # Ensure arrive signal fires only once

# Odometry cache (avoid reading file multiple times per frame)
_cached_odom_data = None
_cached_odom_time = 0
_odom_cache_ttl = 0.05  # seconds (cache for 50ms, max 20 reads/sec)

# Thread safety lock for odometry-based signal state
_odometry_lock = threading.Lock()

_session_ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
detections_filename = os.path.join(DETECTIONS_DIR, f"detections_{_session_ts}.txt")
with open(detections_filename, 'w', encoding='utf-8') as f:
        f.write(f"YOLO Detections Log — session {_session_ts}\n\n")

# SRT file for haptic signals and detections (will be initialized in main)
haptic_srt_filename = None
haptic_srt_counter = 0  # Frame counter for SRT subtitles
haptic_srt_lock = threading.Lock()  # Lock for thread-safe SRT writes
_start_time = None  # Record start time for timestamps
_current_detection_label = threading.local()  # Thread-local storage for current detection label

# Initialize haptic feedback system
def initialize_haptics():
    """Initialize NavHaptics with specified backend"""
    global NAV_HAPTICS
    print("[HAPTIC] ===== INITIALIZING HAPTIC SYSTEM =====", flush=True)
    print("[HAPTIC] Backend: {}".format(BACKEND), flush=True)

    # In pseudo mode: load and generate path data first, before attempting hardware connection
    # This ensures Path.txt is created even if BLE connection fails
    # In live mode: FULL_PATH.txt may not exist yet, so skip this and wait for it later
    if USE_PSEUDO_DATA:
        try:
            print("[HAPTIC] Loading FULL PATH...")
            load_full_path_data()
            generate_path_from_full_path()
            print("[HAPTIC] Loading PATH...")
            check_path()
        except Exception as e:
            print("[HAPTIC] [WARN] Failed to load path data early: {}".format(e))
            # Continue anyway - will try again in backend-specific code

    if BACKEND == 'dry':
        print("[HAPTIC] ===== DRY-RUN MODE =====")
        print("[HAPTIC] Backend: DRY-RUN (UI only, no hardware)")
        try:
            print("[HAPTIC] Initializing NavHaptics with dry backend...")
            nav_haptics_raw = NavHaptics(backend='dry')
            NAV_HAPTICS = NavHapticsWithUI(nav_haptics_raw)
            print("[HAPTIC] [OK] NavHaptics initialized with DRY backend")
            print("[HAPTIC] ========================")
            print("[HAPTIC] ✓ Dry-run ready")
            return True
        except Exception as e:
            print("[HAPTIC] [ERROR] Failed to initialize dry-run: {}".format(e))
            import traceback
            traceback.print_exc()
            return False

    elif BACKEND == 'ble':
        print("[HAPTIC] ===== BLE MODE =====")
        print("[HAPTIC] Backend: BLE (Bluetooth device required)")
        ble_kwargs = {"address": BLE_ADDRESS} if BLE_ADDRESS else {}
        if BLE_ADDRESS:
            print("[HAPTIC] BLE device address (specified): {}".format(BLE_ADDRESS))
        else:
            print("[HAPTIC] BLE auto-scan enabled (looking for 'HapticMotor')")

        try:
            
            print("[HAPTIC] Connecting to BLE device...")
            nav_haptics_raw = NavHaptics(backend='ble', **ble_kwargs)
            print("[HAPTIC] [OK] BLE connection successful!")
            NAV_HAPTICS = NavHapticsWithUI(nav_haptics_raw)
            print("[HAPTIC] [OK] NavHaptics wrapper initialized")
            print("[HAPTIC] ====================")
            # Load path data in live mode (in pseudo mode it was loaded upfront)
            if not USE_PSEUDO_DATA:
                try:
                    print("[HAPTIC] Loading FULL PATH...")
                    load_full_path_data(wait_timeout=10)  # Shorter timeout for live mode
                    generate_path_from_full_path()
                    print("[HAPTIC] Loading PATH...")
                    check_path()
                except Exception as e:
                    print("[HAPTIC] [WARN] Failed to load path data in live mode: {}".format(e))
                    import traceback
                    traceback.print_exc()
            print("[HAPTIC] FULL_PATH loaded: {} chars".format(len(FULL_PATH)), flush=True)
            print("[HAPTIC] PATH loaded: {} commands".format(len(PATH)), flush=True)
            print("[HAPTIC] Signals: START/TURN/ARRIVE detected from ODOMETRY", flush=True)
            if len(PATH) == 0:
                print("[HAPTIC] ⚠️  WARNING: PATH is empty!", flush=True)
            if len(FULL_PATH) == 0:
                print("[HAPTIC] ⚠️  WARNING: FULL_PATH is empty!", flush=True)
            return True
        except Exception as e:
            print("[HAPTIC] [ERROR] BLE connection failed!")
            print("[HAPTIC] Error details: {}".format(e))
            import traceback
            traceback.print_exc()
            print("[HAPTIC] Exiting (BLE backend required, use --backend dry to skip hardware)")
            print("[HAPTIC] ====================")
            return False

    elif BACKEND == 'auto':
        print("[HAPTIC] ===== AUTO MODE =====")
        print("[HAPTIC] Backend: AUTO (try BLE, fall back to dry-run)")

        # Try BLE first
        ble_kwargs = {"address": BLE_ADDRESS} if BLE_ADDRESS else {}
        print("[HAPTIC] Step 1/2: Attempting BLE connection...")
        try:
            print("[HAPTIC]   Connecting to BLE device...")
            nav_haptics_raw = NavHaptics(backend='ble', **ble_kwargs)
            print("[HAPTIC]   [OK] BLE connection successful!")
            NAV_HAPTICS = NavHapticsWithUI(nav_haptics_raw)
            print("[HAPTIC]   [OK] NavHaptics wrapper initialized")
            print("[HAPTIC] Using BLE backend")
            print("[HAPTIC] ====================")
            # Load path data in live mode (in pseudo mode it was loaded upfront)
            if not USE_PSEUDO_DATA:
                try:
                    print("[HAPTIC] Loading FULL PATH...")
                    load_full_path_data(wait_timeout=10)  # Shorter timeout for live mode
                    generate_path_from_full_path()
                    print("[HAPTIC] Loading PATH...")
                    check_path()
                except Exception as e:
                    print("[HAPTIC] [WARN] Failed to load path data in live mode: {}".format(e))
                    import traceback
                    traceback.print_exc()
            return True
        except Exception as e:
            print("[HAPTIC]   [WARN] BLE connection failed: {}".format(e))
            print("[HAPTIC] Step 2/2: Falling back to dry-run mode...")

            # Fall back to dry-run
            try:
                print("[HAPTIC]   Initializing dry-run...")
                nav_haptics_raw = NavHaptics(backend='dry')
                NAV_HAPTICS = NavHapticsWithUI(nav_haptics_raw)
                print("[HAPTIC]   [OK] Dry-run initialized")
                print("[HAPTIC] Using DRY backend (no hardware)")
                print("[HAPTIC] ====================")
                # Load path data in live mode (in pseudo mode it was loaded upfront)
                if not USE_PSEUDO_DATA:
                    try:
                        print("[HAPTIC] Loading FULL PATH...")
                        load_full_path_data(wait_timeout=10)  # Shorter timeout for live mode
                        generate_path_from_full_path()
                        print("[HAPTIC] Loading PATH...")
                        check_path()
                    except Exception as e:
                        print("[HAPTIC] [WARN] Failed to load path data in live mode: {}".format(e))
                        import traceback
                        traceback.print_exc()
                return True
            except Exception as e2:
                print("[HAPTIC]   [ERROR] Failed to initialize dry-run: {}".format(e2))
                import traceback
                traceback.print_exc()
                print("[HAPTIC] ====================")
                return False

    print("[HAPTIC] [ERROR] Unknown backend: {}".format(BACKEND))
    return False

def load_full_path_data(full_path_file="/home/halab2/buzz_shared/path_files/FULL_PATH.txt", wait_timeout=60):
    """
    Load FULL_PATH.txt once at startup and cache waypoint data for arrival detection.

    This is the SINGLE source of truth for the path. All other files are derived from this.
    Extracts both coordinates and heading for predictive turn detection.

    Args:
        full_path_file: Path to FULL_PATH.txt
        wait_timeout: How long to wait for file to exist (seconds)

    Returns:
        (first_waypoint, last_waypoint, expected_distance) or (None, None, None) if failed
    """
    global _arrival_first_waypoint, _arrival_last_waypoint, _arrival_expected_distance, _waypoint_list, _waypoint_data_with_headings, _sim_start_time

    print(f"[LOAD_PATH] Waiting for FULL_PATH.txt ({wait_timeout}s timeout)...")

    # Wait for file to exist
    start_wait = time.time()
    while not os.path.exists(full_path_file):
        elapsed = time.time() - start_wait
        if elapsed > wait_timeout:
            print(f"[LOAD_PATH] ❌ TIMEOUT: FULL_PATH.txt not found within {wait_timeout}s")
            return None, None, None
        remaining = wait_timeout - elapsed
        print(f"[LOAD_PATH] ⏳ Waiting... ({remaining:.0f}s remaining)")
        time.sleep(1)

    print(f"[LOAD_PATH] ✓ FULL_PATH.txt found, reading...")

    try:
        with open(full_path_file, 'r') as f:
            content = f.read()
            lines = content.split('\n') if content.strip() else []

        print(f"[LOAD_PATH] File size: {len(content)} bytes, {len(lines)} lines from FULL_PATH.txt")
        if len(content) > 0 and len(lines) == 0:
            print(f"[LOAD_PATH] DEBUG: First 200 chars of file: {repr(content[:200])}")
        if len(lines) > 0:
            print(f"[LOAD_PATH] DEBUG: First line: {repr(lines[0][:100] if lines[0] else '')}")
            print(f"[LOAD_PATH] DEBUG: Last line: {repr(lines[-1][:100] if lines[-1] else '')}")

        # Extract ALL waypoints for progress tracking
        _waypoint_list = []
        _waypoint_data_with_headings = []
        first_point = None
        last_point = None

        for line in lines:
            if "Point" in line and ":" in line:
                try:
                    # Parse: "Point  0: (  47.650,    2.450) | Heading:   0.0°" or just coordinates
                    parts = line.split("(")[1].split(")")[0].split(",")
                    x = float(parts[0].strip())
                    y = float(parts[1].strip())
                    waypoint = (x, y)
                    _waypoint_list.append(waypoint)

                    # Try to extract heading if present
                    heading_deg = 0.0
                    if "Heading:" in line:
                        try:
                            heading_str = line.split("Heading:")[1].split("°")[0].strip()
                            heading_deg = float(heading_str)
                        except:
                            heading_deg = 0.0

                    waypoint_with_heading = (x, y, heading_deg)
                    _waypoint_data_with_headings.append(waypoint_with_heading)

                    if first_point is None:
                        first_point = waypoint
                    last_point = waypoint
                except:
                    continue

        if first_point is None or last_point is None:
            print(f"[LOAD_PATH] ❌ Failed to extract waypoints from FULL_PATH.txt")
            print(f"[LOAD_PATH] DEBUG: Parsed waypoints: {len(_waypoint_list)}, with headings: {len(_waypoint_data_with_headings)}")
            return None, None, None

        print(f"[LOAD_PATH] ✓ Extracted {len(_waypoint_list)} waypoints for progress tracking")
        print(f"[LOAD_PATH] ✓ Extracted {len(_waypoint_data_with_headings)} waypoints with headings for turn detection")
        if len(_waypoint_data_with_headings) > 0:
            print(f"[LOAD_PATH] First waypoint with heading: {_waypoint_data_with_headings[0]}")
            print(f"[LOAD_PATH] Last waypoint with heading: {_waypoint_data_with_headings[-1]}")

        # Calculate expected distance (single time at startup)
        expected_distance = math.sqrt(
            (last_point[0] - first_point[0])**2 +
            (last_point[1] - first_point[1])**2
        )

        # Cache the values
        _arrival_first_waypoint = first_point
        _arrival_last_waypoint = last_point
        _arrival_expected_distance = expected_distance

        print(f"[LOAD_PATH] ✓ Cached arrival data:")
        print(f"[LOAD_PATH]   First waypoint: {first_point}")
        print(f"[LOAD_PATH]   Last waypoint:  {last_point}")
        print(f"[LOAD_PATH]   Expected distance: {expected_distance:.3f}m")

        # Debug: Show first few waypoints with headings
        if _waypoint_data_with_headings:
            print(f"[LOAD_PATH] ✓ Waypoints with headings (first 10):")
            for i, (x, y, h) in enumerate(_waypoint_data_with_headings[:10]):
                print(f"[LOAD_PATH]   WP {i}: ({x:.3f}, {y:.3f}) heading {h:.1f}°")
            print(f"[LOAD_PATH] ... Waypoints 55-65:")
            for i, (x, y, h) in enumerate(_waypoint_data_with_headings[55:66]):
                print(f"[LOAD_PATH]   WP {55+i}: ({x:.3f}, {y:.3f}) heading {h:.1f}°")
            # CRITICAL DEBUG: Show raw tuple at index 59 right after loading
            if len(_waypoint_data_with_headings) > 59:
                print(f"[LOAD_PATH] CRITICAL: _waypoint_data_with_headings[59] RIGHT AFTER LOAD = {_waypoint_data_with_headings[59]}")

        return first_point, last_point, expected_distance

    except Exception as e:
        print(f"[LOAD_PATH] ❌ Error reading FULL_PATH.txt: {e}")
        import traceback
        traceback.print_exc()
        return None, None, None

def generate_pseudo_path_data():
    """
    Generate ONLY pseudo FULL_PATH.txt (waypoints).
    Path.txt will be derived from FULL_PATH.txt using standard logic.

    This keeps FULL_PATH.txt as the ONLY pseudo input.
    All other files are deterministically derived.

    Creates simulated path:
    - Segment 1: Forward (0,0) → (10,0) heading 0°
    - Segment 2: Left turn 45°
    - Segment 3: Forward 10m heading 45°
    - Segment 4: Left turn 45° (heading 0° → 90°)
    - Segment 5: Forward 10m heading 90°
    """
    path_dir = "/home/halab2/buzz_shared/path_files"
    os.makedirs(path_dir, exist_ok=True)

    # Segment 1: Forward (0,0) to (10,0) heading 0°
    waypoints_seg1 = [(i * 0.5, 0) for i in range(21)]  # 0 to 10m

    # Segment 2: Left turn 45 degrees (from (10,0) to (17.071, 7.071))
    waypoints_seg2 = []
    for angle_deg in range(0, 46, 5):
        angle_rad = math.radians(angle_deg)
        x = 10 + 10 * math.sin(angle_rad)
        y = 10 * (1 - math.cos(angle_rad))
        waypoints_seg2.append((x, y))

    # Segment 3: Forward from end of turn for 10m at 45° heading
    waypoints_seg3 = []
    start_x, start_y = waypoints_seg2[-1]
    turn_heading = math.radians(45)
    for i in range(1, 21):
        distance = i * 0.5
        x = start_x + distance * math.cos(turn_heading)
        y = start_y + distance * math.sin(turn_heading)
        waypoints_seg3.append((x, y))

    # Segment 4: Left turn 45 degrees (from 45° to 90°)
    waypoints_seg4 = []
    start_x, start_y = waypoints_seg3[-1]
    for angle_deg in range(0, 46, 5):  # 0° to 45° left turn
        angle_rad = math.radians(angle_deg)
        # Turn left from 45° heading
        x = start_x + 10 * math.cos(math.radians(45 + angle_deg))
        y = start_y + 10 * math.sin(math.radians(45 + angle_deg))
        waypoints_seg4.append((x, y))

    # Segment 5: Forward from end of turn for 10m at 90° heading
    waypoints_seg5 = []
    start_x, start_y = waypoints_seg4[-1]
    for i in range(1, 21):
        distance = i * 0.5
        x = start_x  # No x change at 90° heading (heading north)
        y = start_y + distance
        waypoints_seg5.append((x, y))

    all_waypoints = waypoints_seg1 + waypoints_seg2[1:] + waypoints_seg3 + waypoints_seg4[1:] + waypoints_seg5

    # Write FULL_PATH.txt (ONLY PSEUDO INPUT)
    full_path_file = os.path.join(path_dir, "FULL_PATH.txt")
    with open(full_path_file, 'w') as f:
        f.write("FULL_PATH.txt - Pseudo test data\n")
        f.write("Total waypoints: {}\n\n".format(len(all_waypoints)))
        for idx, (x, y) in enumerate(all_waypoints):
            f.write("Point {:3d}: ({:8.3f}, {:8.3f})\n".format(idx, x, y))

    print("[PSEUDO_DATA] ✓ Generated FULL_PATH.txt: {} waypoints".format(len(all_waypoints)))
    print("[PSEUDO_DATA]   Segment 1 (FORWARD): 0m to 10m, heading 0°")
    print("[PSEUDO_DATA]   Segment 2 (LEFT TURN): 45° heading change (0° → 45°)")
    print("[PSEUDO_DATA]   Segment 3 (FORWARD): 10m at 45° heading")
    print("[PSEUDO_DATA]   Segment 4 (LEFT TURN): 45° heading change (45° → 90°)")
    print("[PSEUDO_DATA]   Segment 5 (FORWARD): 10m at 90° heading")
    print("[PSEUDO_DATA]   End destination: ({:.2f}, {:.2f})".format(all_waypoints[-1][0], all_waypoints[-1][1]))
    print("[PSEUDO_DATA] Path.txt will be derived from FULL_PATH.txt using standard logic")

    return len(all_waypoints)

def simulate_pseudo_odometry(waypoint_list, simulation_duration=15.0, update_interval=0.5):
    """
    Simulate odometry updates for pseudo mode.
    Makes robot progress along the path over time.

    Args:
        waypoint_list: List of (x, y) waypoints to traverse
        simulation_duration: Total time to reach destination (seconds)
        update_interval: How often to update odometry file (seconds)
    """
    if not waypoint_list or len(waypoint_list) < 2:
        return

    odom_file = '/home/halab2/buzz_shared/path_files/robot_odom.json'
    path_dir = os.path.dirname(odom_file)
    os.makedirs(path_dir, exist_ok=True)

    start_time = time.time()
    first_waypoint = waypoint_list[0]
    last_waypoint = waypoint_list[-1]

    print("[PSEUDO_ODOM] 🤖 Starting odometry simulation")
    print("[PSEUDO_ODOM]    Start: ({:.3f}, {:.3f})".format(first_waypoint[0], first_waypoint[1]))
    print("[PSEUDO_ODOM]    End: ({:.3f}, {:.3f})".format(last_waypoint[0], last_waypoint[1]))
    print("[PSEUDO_ODOM]    Duration: {:.1f}s, Update interval: {:.1f}s".format(
        simulation_duration, update_interval))

    try:
        while True:
            elapsed = time.time() - start_time

            # Stop simulation when duration exceeded
            if elapsed >= simulation_duration:
                # Final position at destination
                odom_data = {
                    "x": last_waypoint[0],
                    "y": last_waypoint[1],
                    "initial_x": first_waypoint[0],
                    "initial_y": first_waypoint[1],
                    "yaw": 1.5708,  # 90 degrees (π/2 rad) - heading north after two left turns
                    "timestamp": datetime.now().isoformat(),
                    "status": "arrived"
                }
                with open(odom_file, 'w') as f:
                    json.dump(odom_data, f)
                print("[PSEUDO_ODOM] ✓ Simulation complete - final position set")
                break

            # Interpolate position along path
            progress = min(1.0, elapsed / simulation_duration)  # 0 to 1

            # Find which two waypoints to interpolate between
            waypoint_index = progress * (len(waypoint_list) - 1)
            wp_idx1 = int(waypoint_index)
            wp_idx2 = min(wp_idx1 + 1, len(waypoint_list) - 1)
            local_progress = waypoint_index - wp_idx1

            # Interpolate between waypoints
            wp1 = waypoint_list[wp_idx1]
            wp2 = waypoint_list[wp_idx2]

            x = wp1[0] + (wp2[0] - wp1[0]) * local_progress
            y = wp1[1] + (wp2[1] - wp1[1]) * local_progress

            # Simulate heading changes for all turns:
            # Segment 1 (0-26%): 0° heading
            # Segment 2 (26-33%): Turn left from 0° to 45°
            # Segment 3 (33-60%): 45° heading
            # Segment 4 (60-67%): Turn left from 45° to 90°
            # Segment 5 (67-100%): 90° heading
            yaw = 0.0  # Start heading
            if progress > 0.26 and progress <= 0.33:  # First left turn (0° → 45°)
                turn_progress = (progress - 0.26) / (0.33 - 0.26)
                yaw = turn_progress * 0.785  # Gradually turn to 45° (0.785 rad)
            elif progress > 0.33 and progress <= 0.60:  # Straight at 45°
                yaw = 0.785  # Full 45° heading
            elif progress > 0.60 and progress <= 0.67:  # Second left turn (45° → 90°)
                turn_progress = (progress - 0.60) / (0.67 - 0.60)
                yaw = 0.785 + turn_progress * 0.785  # Gradually turn from 45° to 90°
            elif progress > 0.67:  # Straight at 90°
                yaw = 1.5708  # 90° heading (π/2 rad)
            # else: 0° heading (Segment 1)

            # Write odometry update
            odom_data = {
                "x": x,
                "y": y,
                "initial_x": first_waypoint[0],
                "initial_y": first_waypoint[1],
                "yaw": min(0.785, yaw),
                "timestamp": datetime.now().isoformat(),
                "progress": progress * 100,
                "waypoint_index": wp_idx1
            }

            with open(odom_file, 'w') as f:
                json.dump(odom_data, f)

            # Sleep until next update
            time.sleep(update_interval)

    except Exception as e:
        print("[PSEUDO_ODOM] ❌ Error in odometry simulation: {}".format(e))
        import traceback
        traceback.print_exc()
        

def generate_path_from_full_path(full_path_file="/home/halab2/buzz_shared/path_files/FULL_PATH.txt"):
    """
    Generate Path.txt from FULL_PATH.txt using standard waypoint-to-command conversion.

    This ensures Path.txt is derived from FULL_PATH.txt (not a separate pseudo input).
    Works for both real and pseudo paths.

    Args:
        full_path_file: Path to FULL_PATH.txt

    Returns:
        Number of commands written to Path.txt, or 0 if failed
    """
    path_dir = os.path.dirname(full_path_file)

    try:
        # Read FULL_PATH.txt
        if not os.path.exists(full_path_file):
            print("[PATH_DERIVE] ❌ FULL_PATH.txt not found, cannot derive Path.txt")
            return 0

        with open(full_path_file, 'r') as f:
            lines = f.readlines()

        # Extract waypoints
        waypoints = []
        for line in lines:
            if "Point" in line and ":" in line:
                try:
                    parts = line.split("(")[1].split(")")[0].split(",")
                    x = float(parts[0].strip())
                    y = float(parts[1].strip())
                    waypoints.append((x, y))
                except:
                    continue

        if not waypoints:
            print("[PATH_DERIVE] ❌ No waypoints found in FULL_PATH.txt")
            return 0

        print("[PATH_DERIVE] ✓ Read {} waypoints from FULL_PATH.txt".format(len(waypoints)))

        # Generate Path.txt with forward commands for each waypoint segment
        # One command per waypoint = forward movement between waypoints
        path_file = os.path.join(path_dir, "Path.txt")
        start_time = datetime.now()
        total_commands = 0

        with open(path_file, 'w') as f:
            f.write("FULL PATH: {}\n".format(len(waypoints)))

            # Generate forward commands for most waypoints
            # (excluding last, which gets arrive command)
            for i in range(len(waypoints) - 1):
                cmd_time = start_time + timedelta(seconds=i)
                f.write("PATH: forward, time: {}\n".format(cmd_time.strftime("%Y%m%d_%H%M%S")))
                total_commands += 1

            # Final command: arrive at destination
            cmd_time = start_time + timedelta(seconds=len(waypoints) - 1)
            f.write("PATH: arrive, time: {}\n".format(cmd_time.strftime("%Y%m%d_%H%M%S")))
            total_commands += 1

            # Ensure file is flushed to disk
            f.flush()
            os.fsync(f.fileno())

        # Verify file was written
        if os.path.exists(path_file):
            file_size = os.path.getsize(path_file)
            print("[PATH_DERIVE] ✓ Generated Path.txt: {} commands (file size: {} bytes)".format(total_commands, file_size))
        else:
            print("[PATH_DERIVE] ⚠️  File was not saved to disk!")

        return total_commands

    except Exception as e:
        print("[PATH_DERIVE] ❌ Error generating Path.txt: {}".format(e))
        import traceback
        traceback.print_exc()
        return 0



def init_haptic_srt():
    """Initialize SRT file with header"""
    global haptic_srt_filename, _start_time, haptic_srt_counter
    _start_time = datetime.now()
    haptic_srt_filename = os.path.join(HAPTIC_DIR, f"haptic_signals_{_session_ts}.srt")
    haptic_srt_counter = 1  # Start at 1 since we're adding a header entry

    try:
        with open(haptic_srt_filename, 'w', encoding='utf-8') as f:
            f.write("1\n00:00:00,000 --> 00:00:00,000\nSYSTEM: Session started - Camera and haptic system ready\n\n")
        print("[SRT] Initialized: {}".format(haptic_srt_filename), flush=True)
        print("[SRT] Waiting for UI to emit START signal...", flush=True)
    except Exception as e:
        print("[SRT] Error initializing file: {}".format(e), flush=True)


def get_srt_timestamp():
    """Get current elapsed time as SRT timestamp (HH:MM:SS,mmm)"""
    if _start_time is None:
        return "00:00:00,000"
    elapsed = datetime.now() - _start_time
    total_seconds = int(elapsed.total_seconds())
    milliseconds = int((elapsed.total_seconds() - total_seconds) * 1000)
    hours = total_seconds // 3600
    minutes = (total_seconds % 3600) // 60
    seconds = total_seconds % 60
    return "{:02d}:{:02d}:{:02d},{:03d}".format(hours, minutes, seconds, milliseconds)

def write_detection_to_srt(detection_text, zone=None):
    """Write a detection entry to the SRT file with optional zone info"""
    global haptic_srt_counter, haptic_srt_filename
    with haptic_srt_lock:
        try:
            timestamp = get_srt_timestamp()
            haptic_srt_counter += 1
            if zone:
                entry = f"{haptic_srt_counter}\n{timestamp} --> {timestamp}\nDETECTION: {detection_text} | ZONE: {zone}\n\n"
            else:
                entry = f"{haptic_srt_counter}\n{timestamp} --> {timestamp}\nDETECTION: {detection_text}\n\n"
            with open(haptic_srt_filename, 'a', encoding='utf-8') as f:
                f.write(entry)
            print("[SRT] Detection logged: {}".format(detection_text[:60]), flush=True)
        except Exception as e:
            print("[SRT] Error writing detection: {}".format(e), flush=True)

def write_emit_debug_to_srt(debug_text):
    """Write emit_haptic debug info to SRT file"""
    global haptic_srt_counter, haptic_srt_filename

    if haptic_srt_filename is None:
        return

    with haptic_srt_lock:
        try:
            timestamp = get_srt_timestamp()
            haptic_srt_counter += 1
            entry = f"{haptic_srt_counter}\n{timestamp} --> {timestamp}\nDEBUG: {debug_text}\n\n"
            with open(haptic_srt_filename, 'a', encoding='utf-8') as f:
                f.write(entry)
        except Exception as e:
            print("[SRT] Error writing debug: {}".format(e), flush=True)

def write_signal_to_srt(signal_name, motors=None, intensity=None):
    """Write a signal firing entry to the SRT file"""
    global haptic_srt_counter, haptic_srt_filename

    # Get detection label from thread-local storage if available
    detection_label = getattr(_current_detection_label, 'label', None)

    print("[SRT] write_signal_to_srt called: signal={}, motors={}, intensity={}, detection={}".format(
        signal_name, motors, intensity, detection_label[:50] if detection_label else None), flush=True)

    if haptic_srt_filename is None:
        print("[SRT] ERROR: haptic_srt_filename is None, SRT not initialized", flush=True)
        return

    with haptic_srt_lock:
        try:
            timestamp = get_srt_timestamp()
            haptic_srt_counter += 1

            signal_text = "SIGNAL: {}".format(signal_name)
            if motors is not None:
                signal_text += " (motors: {})".format(motors)
            if intensity is not None:
                signal_text += " [intensity: {:.2f}]".format(intensity)

            # Include detection info if available
            if detection_label:
                signal_text += "\nDETECTION: {}".format(detection_label)

            entry = f"{haptic_srt_counter}\n{timestamp} --> {timestamp}\n{signal_text}\n\n"
            with open(haptic_srt_filename, 'a', encoding='utf-8') as f:
                f.write(entry)
            print("[SRT] ✓ Signal logged to {}: {}".format(haptic_srt_filename, signal_text[:80]), flush=True)
        except Exception as e:
            print("[SRT] ✗ Error writing signal: {}".format(e), flush=True)
            import traceback
            traceback.print_exc()

def write_waypoint_to_srt(wp_idx, target_x, target_y, odom_x, odom_y, heading, progress_pct, distance_remaining):
    """Write waypoint progress to SRT file (live mode)"""
    global haptic_srt_counter, haptic_srt_filename

    if haptic_srt_filename is None:
        return

    with haptic_srt_lock:
        try:
            timestamp = get_srt_timestamp()
            haptic_srt_counter += 1
            entry = f"{haptic_srt_counter}\n{timestamp} --> {timestamp}\n[WAYPOINT] WP {wp_idx} | Target: ({target_x:.3f}, {target_y:.3f}) | Odom: ({odom_x:.3f}, {odom_y:.3f}) | Heading: {heading:.1f}° | {progress_pct:.1f}% | {distance_remaining:.1f}m remaining\n\n"
            with open(haptic_srt_filename, 'a', encoding='utf-8') as f:
                f.write(entry)
        except Exception as e:
            print("[SRT] Error writing waypoint: {}".format(e), flush=True)

def write_turn_to_srt(turn_direction, odom_x, odom_y, heading, current_direction=None, next_direction=None, heading_change=None):
    """Write turn detection to SRT file (live mode)

    Args:
        turn_direction: "left" or "right"
        odom_x, odom_y: Robot odometry position
        heading: Heading in degrees (numeric)
        current_direction: Current cardinal direction (e.g., "North", "East")
        next_direction: Next cardinal direction after turn
        heading_change: Magnitude of heading change in degrees
    """
    global haptic_srt_counter, haptic_srt_filename

    if haptic_srt_filename is None:
        return

    with haptic_srt_lock:
        try:
            timestamp = get_srt_timestamp()
            haptic_srt_counter += 1
            signal_type = "TURN_LEFT" if turn_direction == "left" else "TURN_RIGHT"

            # Build entry with cardinal direction info if provided
            if current_direction and next_direction:
                entry = f"{haptic_srt_counter}\n{timestamp} --> {timestamp}\n[SIGNAL] {signal_type} (predictive) | Heading: {current_direction} → {next_direction}"
                if heading_change is not None:
                    entry += f" ({heading_change:.1f}° change)"
                entry += f" | Odom: ({odom_x:.3f}, {odom_y:.3f})\n\n"
            else:
                # Fallback for non-predictive turns
                entry = f"{haptic_srt_counter}\n{timestamp} --> {timestamp}\n[SIGNAL] {signal_type} detected | Odom: ({odom_x:.3f}, {odom_y:.3f}) | Heading: {heading:.1f}°\n\n"

            with open(haptic_srt_filename, 'a', encoding='utf-8') as f:
                f.write(entry)
        except Exception as e:
            print("[SRT] Error writing turn: {}".format(e), flush=True)

def write_arrive_to_srt(odom_x, odom_y, heading):
    """Write arrival detection to SRT file (live mode)"""
    global haptic_srt_counter, haptic_srt_filename

    if haptic_srt_filename is None:
        return

    with haptic_srt_lock:
        try:
            timestamp = get_srt_timestamp()
            haptic_srt_counter += 1
            entry = f"{haptic_srt_counter}\n{timestamp} --> {timestamp}\n[SIGNAL] ARRIVE detected | Odom: ({odom_x:.3f}, {odom_y:.3f}) | Heading: {heading:.1f}°\n\n"
            with open(haptic_srt_filename, 'a', encoding='utf-8') as f:
                f.write(entry)
        except Exception as e:
            print("[SRT] Error writing arrive: {}".format(e), flush=True)

def log_all_signals_to_srt():
    """
    Comprehensive signal logging function that writes ALL signals to SRT.
    This is called by ALL signal emission points to ensure nothing is missed.
    """
    global haptic_srt_counter, haptic_srt_filename

    if haptic_srt_filename is None:
        print("[SRT] WARNING: SRT not initialized yet", flush=True)
        return False

    return True

def emit_haptic_signal(signal_name, signal_func, debug_label):
    """
    Unified haptic signal emission with consistent error handling.

    Args:
        signal_name: Name of signal (e.g., "START", "TURN_LEFT", "ARRIVE")
        signal_func: Callable from NAV_HAPTICS (e.g., NAV_HAPTICS.start)
        debug_label: Human-readable label for SRT logging
    """
    try:
        print("[EMIT_HAPTIC] 🔊 {} signal firing!".format(signal_name))
        signal_func()
        print("[EMIT_HAPTIC] {} SUCCESS".format(signal_name))
        write_emit_debug_to_srt(debug_label)
        return True
    except Exception as e:
        print("[EMIT_HAPTIC] {} FAILED: {}".format(signal_name, str(e)))
        return False

def read_odom_cached():
    """
    Read odometry with caching to avoid excessive file I/O.
    Caches for 50ms (max ~20 reads/sec instead of 60+).

    Returns:
        odom_data dict or None
    """
    global _cached_odom_data, _cached_odom_time

    current_time = time.time()
    if _cached_odom_data is not None and (current_time - _cached_odom_time) < _odom_cache_ttl:
        return _cached_odom_data  # Use cached data

    try:
        odom_file = '/home/halab2/buzz_shared/path_files/robot_odom.json'
        if not os.path.exists(odom_file):
            print("[ODOM] ⚠️  Odometry file not found: {}".format(odom_file), flush=True)
            return None

        with open(odom_file, 'r') as f:
            _cached_odom_data = json.load(f)
            _cached_odom_time = current_time
            print("[ODOM] ✓ Read odometry: x={}, y={}, yaw={}".format(
                _cached_odom_data.get('x', 'N/A'),
                _cached_odom_data.get('y', 'N/A'),
                _cached_odom_data.get('yaw', 'N/A')), flush=True)
            return _cached_odom_data
    except (FileNotFoundError, json.JSONDecodeError, IOError) as e:
        print("[ODOM] ❌ Error reading odometry: {}".format(e), flush=True)
        return None

def detect_start_from_odometry(movement_threshold=0.1):
    """
    Detect when robot starts moving from initial position.

    Uses real odometry data to know when motion actually begins.

    Args:
        movement_threshold: Distance in meters robot must move to trigger start (default 0.1m)

    Returns:
        True if start signal should fire, False otherwise
    """
    global _initial_position, _start_signal_fired

    with _odometry_lock:
        if _start_signal_fired:
            return False  # Already fired
        odom_data = read_odom_cached()
        if odom_data is None:
            return False

        try:
            current_x = odom_data['x']
            current_y = odom_data['y']
        except KeyError:
            return False  # Missing required fields

        # Initialize on first call
        if _initial_position is None:
            _initial_position = (current_x, current_y)
            print("[START] Initial position: ({:.3f}, {:.3f})".format(current_x, current_y))
            return False

        # Calculate distance traveled
        distance_moved = math.sqrt(
            (current_x - _initial_position[0])**2 +
            (current_y - _initial_position[1])**2
        )

        # Check if movement threshold exceeded
        if distance_moved >= movement_threshold:
            _start_signal_fired = True
            print("[START] ✓ Movement detected! Distance: {:.3f}m → START SIGNAL".format(distance_moved))
            return True

        return False

def detect_turn_from_odometry(turn_threshold):
    """
    Detect left/right turns from actual robot heading (yaw) changes.

    Uses real odometry data instead of pre-planned path.

    Args:
        turn_threshold: Heading change in degrees to count as a turn (default 45°)

    Returns:
        "left", "right", or None
    """
    global _initial_heading, _previous_heading, _last_turn_emission_time

    with _odometry_lock:
        odom_data = read_odom_cached()
        if odom_data is None:
            print("[TURN] ❌ No odometry data available", flush=True)
            return None

        try:
            current_heading = odom_data['yaw']  # Radians from ROS
        except KeyError as e:
            print("[TURN] ❌ Missing yaw field in odometry: {}".format(e), flush=True)
            print("[TURN] Available fields: {}".format(list(odom_data.keys())), flush=True)
            return None  # Missing yaw field

        # Initialize on first call
        if _initial_heading is None:
            _initial_heading = current_heading
            _previous_heading = current_heading
            print("[TURN] ✓ Initial heading set: {:.2f}° ({:.3f} rad)".format(
                math.degrees(current_heading), current_heading), flush=True)
            return None

        # Calculate heading change since last check
        heading_change = current_heading - _previous_heading
        heading_change_deg_before_norm = math.degrees(heading_change)

        # Normalize to [-π, π]
        while heading_change > math.pi:
            heading_change -= 2 * math.pi
        while heading_change < -math.pi:
            heading_change += 2 * math.pi

        heading_change_deg = math.degrees(heading_change)

        # ALWAYS update previous heading (fixes drift accumulation bug)
        _previous_heading = current_heading

        # Check if we should emit a turn signal (with cooldown to prevent spam)
        current_time = time.time()
        if current_time - _last_turn_emission_time < _turn_emit_cooldown:
            return None  # Still in cooldown

        # Detect turn
        if heading_change_deg > turn_threshold:
            _last_turn_emission_time = current_time
            print("[TURN] 🔄 LEFT TURN detected! Heading change: +{:.1f}°".format(heading_change_deg))
            return "left"
        elif heading_change_deg < -turn_threshold:
            _last_turn_emission_time = current_time
            print("[TURN] 🔄 RIGHT TURN detected! Heading change: {:.1f}°".format(heading_change_deg))
            return "right"

        return None

def track_waypoint_progress():
    """
    Track robot progress through waypoints and log when reaching each one.
    Called every frame to monitor waypoint-by-waypoint movement.

    Returns:
        (current_waypoint_index, progress_percent, distance_to_destination)
    """
    global _waypoint_list, _last_reported_waypoint, _arrival_expected_distance, _sim_start_time

    if not _waypoint_list or _arrival_expected_distance is None:
        return None, None, None

    if _sim_start_time is None:
        _sim_start_time = time.time()

    try:
        # Get current position from odometry
        odom_data = read_odom_cached()
        if odom_data is None:
            return None, None, None

        current_x = odom_data['x']
        current_y = odom_data['y']
        initial_x = odom_data['initial_x']
        initial_y = odom_data['initial_y']

        # Calculate actual distance traveled
        actual_distance = math.sqrt(
            (current_x - initial_x)**2 +
            (current_y - initial_y)**2
        )

        # Find closest waypoint to current position
        current_waypoint_idx = -1
        min_distance_to_waypoint = float('inf')

        for idx, waypoint in enumerate(_waypoint_list):
            distance_to_wp = math.sqrt(
                (current_x - waypoint[0])**2 +
                (current_y - waypoint[1])**2
            )
            if distance_to_wp < min_distance_to_waypoint:
                min_distance_to_waypoint = distance_to_wp
                current_waypoint_idx = idx

        # Calculate progress
        progress_percent = (actual_distance / _arrival_expected_distance) * 100
        distance_to_destination = max(0, _arrival_expected_distance - actual_distance)

        # Log when reaching a new waypoint
        if current_waypoint_idx > _last_reported_waypoint and current_waypoint_idx >= 0:
            _last_reported_waypoint = current_waypoint_idx
            elapsed = time.time() - _sim_start_time
            wp_x, wp_y = _waypoint_list[current_waypoint_idx]

            print(f"[WAYPOINT] 📍 Waypoint {current_waypoint_idx}/{len(_waypoint_list)-1} reached!")
            print(f"[WAYPOINT]    Position: ({wp_x:.3f}, {wp_y:.3f})")
            print(f"[WAYPOINT]    Distance traveled: {actual_distance:.3f}m / {_arrival_expected_distance:.3f}m ({progress_percent:.1f}%)")
            print(f"[WAYPOINT]    Distance remaining: {distance_to_destination:.3f}m")
            print(f"[WAYPOINT]    Time elapsed: {elapsed:.1f}s")
            print(f"[WAYPOINT]    Closest waypoint: {current_waypoint_idx} (distance: {min_distance_to_waypoint:.3f}m away)", flush=True)

        return current_waypoint_idx, progress_percent, distance_to_destination

    except Exception as e:
        return None, None, None

def detect_arrive_from_odometry():
    """
    Detect arrival when robot reaches end of path.

    Compares actual distance traveled (from odometry) to expected distance.
    Expected distance is cached at startup from FULL_PATH.txt (single read operation).

    Returns:
        True if robot has arrived (actual_distance >= expected_distance), False otherwise
    """
    global _arrive_signal_fired, _arrival_expected_distance, _initial_position

    # Reset arrive flag if robot has returned to start (new run detected)
    if _arrive_signal_fired and _initial_position is not None:
        try:
            odom_data = read_odom_cached()
            if odom_data:
                current_x = odom_data.get('x', 0)
                current_y = odom_data.get('y', 0)
                distance_from_start = math.sqrt(
                    (current_x - _initial_position[0])**2 +
                    (current_y - _initial_position[1])**2
                )
                if distance_from_start < 0.5:  # Robot back at start = new run
                    _arrive_signal_fired = False
                    print("[ARRIVE] 🔄 New run detected (robot at start), resetting arrive flag", flush=True)
        except:
            pass

    if _arrive_signal_fired:
        return False

    # Check if cached distance is available
    if _arrival_expected_distance is None:
        print("[ARRIVE] ❌ Cached arrival data not loaded", flush=True)
        return False

    try:
        # Get actual distance from odometry
        odom_data = read_odom_cached()
        if odom_data is None:
            print("[ARRIVE] ❌ Odometry file not available", flush=True)
            return False

        try:
            current_x = odom_data['x']
            current_y = odom_data['y']
            initial_x = odom_data['initial_x']
            initial_y = odom_data['initial_y']
        except KeyError as e:
            print("[ARRIVE] DEBUG: Missing odometry field: {}".format(e))
            return False

        # Calculate actual distance traveled from odometry
        actual_distance = math.sqrt(
            (current_x - initial_x)**2 +
            (current_y - initial_y)**2
        )

        # Compare: actual vs cached expected distance
        print("[ARRIVE] DEBUG: actual_distance={:.3f}m, expected_distance={:.3f}m, comparison: {:.3f} >= {:.3f}?".format(
            actual_distance, _arrival_expected_distance, actual_distance, _arrival_expected_distance))

        # Check if arrived for each condition
        if CONDITION == 1:
            pre_stop_distance = _arrival_expected_distance -  1 # meters out
        else:
            pre_stop_distance = _arrival_expected_distance -  2 # meters out

        if actual_distance >= pre_stop_distance and not _arrive_signal_fired:
            _arrive_signal_fired = True
            print("[ARRIVE] ⚠️  Approaching destination! Distance: {:.3f}m / {:.3f}m expected".format(
                actual_distance, _arrival_expected_distance))
            return True
        
        if actual_distance >= _arrival_expected_distance:
            print("[ARRIVE] ⭐ DESTINATION REACHED! Distance: {:.3f}m / {:.3f}m expected".format(
                actual_distance, _arrival_expected_distance))
            return True

        return False

    except Exception as e:
        print("[ARRIVE] ⚠️  Error detecting arrival: {}".format(e))
        import traceback
        traceback.print_exc()
        return False

def get_cardinal_direction(heading_deg):
    """
    Classify heading into one of 4 cardinal directions: N, S, E, W.

    Coordinate system: North=0°, West=90°, East=-90°, South=±180°
    Each direction has ±25° range.

    Args:
        heading_deg: Heading in degrees (-180 to 180)

    Returns:
        'North', 'West', 'East', 'South', or None if not in any range
    """
    heading = heading_deg  # Use heading as-is, no normalization

    for direction, (min_deg, max_deg) in CARDINAL_RANGES.items():
        # Handle South which spans positive and negative sides
        if direction == 'South_Pos':
            if min_deg <= heading <= max_deg:  # 155 to 180
                return 'South'
        elif direction == 'South_Neg':
            if min_deg <= heading <= max_deg:  # -180 to -155
                return 'South'
        # Handle North, West, East normally
        elif min_deg <= heading <= max_deg:
            return direction

    # Not in any range - this is expected for headings in gaps (25-65, 115-155, -155 to -115, -65 to -25)
    return None

def get_turn_direction(current_direction, next_direction):
    """
    Determine LEFT or RIGHT turn based on cardinal direction change.

    Compass-style directions follow clockwise order: North → East → South → West → North
    - Moving forward in this order (1-2 steps) = clockwise = RIGHT turn
    - Moving backward in this order (3 steps / -1) = counterclockwise = LEFT turn

    Args:
        current_direction: Current cardinal direction ('North', 'East', 'South', 'West')
        next_direction: Next cardinal direction

    Returns:
        'RIGHT' (clockwise), 'LEFT' (counterclockwise), or None if same direction
    """
    direction_order = ['North', 'East', 'South', 'West']

    if current_direction not in direction_order or next_direction not in direction_order:
        return None

    if current_direction == next_direction:
        return None  # No turn, same direction

    current_idx = direction_order.index(current_direction)
    next_idx = direction_order.index(next_direction)

    # Calculate distance forward (clockwise)
    distance_forward = (next_idx - current_idx) % 4

    # Forward 1 or 2 steps = RIGHT (clockwise)
    # Forward 3 steps (same as backward 1) = LEFT (counterclockwise)
    if distance_forward in [1, 2]:
        return "RIGHT"
    else:  # distance_forward == 3
        return "LEFT"

def normalize_heading_delta(delta):
    """Normalize heading delta to -180 to 180 range."""
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    return delta

def circular_mean_heading(headings):
    """Calculate circular mean of heading angles (handles ±180° wraparound)."""
    if not headings:
        return 0.0
    sin_sum = sum(math.sin(math.radians(h)) for h in headings)
    cos_sum = sum(math.cos(math.radians(h)) for h in headings)
    mean_rad = math.atan2(sin_sum, cos_sum)
    mean_deg = math.degrees(mean_rad)
    while mean_deg > 180:
        mean_deg -= 360
    while mean_deg < -180:
        mean_deg += 360
    return mean_deg

def predict_upcoming_turn(current_waypoint_idx):
    """
    Detect upcoming turns using window-averaged heading comparison.

    Compares average heading of a trailing window (where we are) to a leading window
    (where we're going). If the net heading change across both windows exceeds threshold,
    it's a real turn. This approach correctly handles ±180° wraparound and catches
    gradual turns spread over many points without false positives from micro-jitter.

    Args:
        current_waypoint_idx: Current position in waypoint list

    Returns:
        Dict with turn info or None if no real turn found:
        {
            'turn_type': 'LEFT' or 'RIGHT',
            'current_direction': 'North', 'East', etc.,
            'next_direction': 'North', 'East', etc.,
            'turn_waypoint_index': index of turn waypoint,
            'distance_to_turn': distance in meters
        }
    """
    global _waypoint_data_with_headings, _sustained_turn_sequences

    if not _waypoint_data_with_headings:
        return None

    if current_waypoint_idx < 0 or current_waypoint_idx >= len(_waypoint_data_with_headings):
        return None

    # Check if this waypoint is already part of a detected turn
    for turn_start_idx, turn_info in _sustained_turn_sequences.items():
        if turn_start_idx <= current_waypoint_idx <= turn_info.get('last_window_idx', turn_start_idx):
            return None

    # Need enough waypoints ahead for trailing + leading windows
    min_required = WINDOW_SIZE * 2
    if current_waypoint_idx + min_required > len(_waypoint_data_with_headings):
        return None

    # Trailing window: average heading from current to current+WINDOW_SIZE
    trailing_headings = [_waypoint_data_with_headings[i][2] for i in range(current_waypoint_idx, current_waypoint_idx + WINDOW_SIZE)]
    trailing_avg = circular_mean_heading(trailing_headings)
    trailing_cardinal = get_cardinal_direction(trailing_avg)

    if trailing_cardinal is None:
        return None

    # Leading window: average heading from current+WINDOW_SIZE to current+WINDOW_SIZE*2
    leading_headings = [_waypoint_data_with_headings[i][2] for i in range(current_waypoint_idx + WINDOW_SIZE, current_waypoint_idx + WINDOW_SIZE * 2)]
    leading_avg = circular_mean_heading(leading_headings)
    leading_cardinal = get_cardinal_direction(leading_avg)

    if leading_cardinal is None:
        return None

    # Calculate net heading change between windows
    heading_delta = normalize_heading_delta(leading_avg - trailing_avg)

    if abs(heading_delta) < HEADING_CHANGE_THRESHOLD:
        return None

    # Determine turn type
    turn_type = get_turn_direction(trailing_cardinal, leading_cardinal)
    if turn_type is None:
        return None

    # Calculate distance from current to end of leading window
    turn_waypoint_idx = current_waypoint_idx + WINDOW_SIZE * 2 - 1
    dist_to_turn = 0.0
    for i in range(current_waypoint_idx, turn_waypoint_idx):
        wp1 = _waypoint_data_with_headings[i]
        wp2 = _waypoint_data_with_headings[i + 1]
        segment_dist = math.sqrt((wp2[0] - wp1[0])**2 + (wp2[1] - wp1[1])**2)
        dist_to_turn += segment_dist

    turn_info = {
        'turn_type': turn_type,
        'current_direction': trailing_cardinal,
        'next_direction': leading_cardinal,
        'turn_waypoint_index': turn_waypoint_idx,
        'distance_to_turn': dist_to_turn,
        'heading_delta': heading_delta,
        'last_window_idx': current_waypoint_idx + WINDOW_SIZE * 2 - 1
    }

    # Track to avoid re-detecting
    _sustained_turn_sequences[current_waypoint_idx] = turn_info

    print("[PREDICT_TURN] ✓ Real turn detected! {} → {} (Δ{:+.1f}°, {:.2f}m away)".format(
        trailing_cardinal, leading_cardinal, heading_delta, dist_to_turn), flush=True)

    return turn_info

def emit_predictive_turn_signal(turn_info, current_pos):
    """
    Emit TURN_LEFT or TURN_RIGHT signal when within distance threshold of upcoming turn.

    Tracks signaled turns to avoid duplicate signals. For overlapping sustained turn detections
    (e.g., same physical turn detected at multiple waypoints), only signals once.

    Args:
        turn_info: Dict from predict_upcoming_turn()
        current_pos: (x, y) tuple from odometry

    Returns:
        True if signal was emitted, False otherwise
    """
    global _signaled_turn_indices, _sustained_turn_sequences

    if turn_info is None:
        return False

    turn_wp_idx = turn_info['turn_waypoint_index']

    # Check for de-duplication: if this turn overlaps with a previously detected sustained turn sequence,
    # only signal the first detection
    for prev_turn_start_idx, prev_turn_info in _sustained_turn_sequences.items():
        # Skip the current turn info itself
        if prev_turn_start_idx == turn_wp_idx:
            continue

        # Check if turns overlap (within 5 waypoints of each other)
        prev_turn_range = range(prev_turn_start_idx, min(prev_turn_start_idx + 10, len(_waypoint_data_with_headings)))
        if turn_wp_idx in prev_turn_range:
            # This is an overlapping detection of the same turn
            if prev_turn_start_idx < turn_wp_idx:
                # Earlier detection was already made, skip this one
                print("[PREDICT_TURN] Skipping overlapping turn at WP {} (already detected at WP {})".format(
                    turn_wp_idx, prev_turn_start_idx), flush=True)
                return False

    # Skip if already signaled this turn
    if turn_wp_idx in _signaled_turn_indices:
        print("[PREDICT_TURN] ⚠️  Turn at WP {} already signaled, skipping".format(turn_wp_idx), flush=True)
        return False

    distance_to_turn = turn_info['distance_to_turn']

    print("[PREDICT_TURN] DEBUG: Checking signal for {} turn {} → {} at {:.2f}m (Δ{:+.1f}°)".format(
        turn_info['turn_type'],
        turn_info['current_direction'],
        turn_info['next_direction'],
        distance_to_turn,
        turn_info.get('heading_delta', 0)), flush=True)

    # Check if within signal threshold
    print("[PREDICT_TURN] DEBUG: Is {:.2f}m < {:.2f}m threshold? {}".format(
        distance_to_turn, SIGNAL_DISTANCE_THRESHOLD, distance_to_turn < SIGNAL_DISTANCE_THRESHOLD), flush=True)

    if distance_to_turn < SIGNAL_DISTANCE_THRESHOLD:
        signal_type = "TURN_LEFT" if turn_info['turn_type'] == "LEFT" else "TURN_RIGHT"

        print("[PREDICT_TURN] 🔄 {} signal! {} → {} in {:.2f}m".format(
            signal_type,
            turn_info['current_direction'],
            turn_info['next_direction'],
            distance_to_turn), flush=True)

        # Mark this turn as signaled
        _signaled_turn_indices.add(turn_wp_idx)

        # Emit the haptic signal
        emit_haptic_feedback(signal_type.lower().replace('_', ' '))

        # Log to SRT with cardinal direction info
        write_turn_to_srt(
            turn_direction=turn_info['turn_type'].lower(),
            odom_x=current_pos[0],
            odom_y=current_pos[1],
            heading=0.0,
            current_direction=turn_info['current_direction'],
            next_direction=turn_info['next_direction'],
            heading_change=None
        )

        return True

    return False

def calculate_robot_waypoint_progress(path_length=None, robot_speed=1.0):
    """
    Calculate robot's current waypoint position based on ACTUAL ODOMETRY distance.

    Uses real position data from buzz_cam_haptic.py instead of time estimation.

    Args:
        path_length: Total path length in meters (auto-calculated from PATH if None)
        robot_speed: Robot speed in m/s (not used when odometry available)

    Returns:
        Current waypoint index (0 to len(PATH)-1) or distance traveled in meters
    """
    global _robot_waypoint_progress, _last_progress_update_time, PATH

    # Auto-calculate path_length from actual PATH if not provided
    if path_length is None:
        import math
        from datetime import datetime
        path_length = 0

        # Extract timestamps from PATH to calculate actual duration
        if len(PATH) >= 2:
            try:
                # Extract time from first and last commands
                # Format: "forward, time: 20260807_033231"
                first_time_str = PATH[0].split("time: ")[1] if "time: " in PATH[0] else None
                last_time_str = PATH[-1].split("time: ")[1] if "time: " in PATH[-1] else None

                if first_time_str and last_time_str:
                    # Parse timestamps
                    first_time = datetime.strptime(first_time_str, "%Y%m%d_%H%M%S")
                    last_time = datetime.strptime(last_time_str, "%Y%m%d_%H%M%S")

                    # Calculate actual duration in seconds
                    duration = (last_time - first_time).total_seconds()

                    # path_length = speed * duration
                    path_length = robot_speed * duration

                    print("[PROGRESS] Calculated path_length from timestamps: {:.2f}s duration → {:.2f}m at {:.2f}m/s".format(
                        duration, path_length, robot_speed))
                else:
                    # Fallback if timestamps not found
                    path_length = len(PATH) * 0.5
                    print("[PROGRESS] Could not parse timestamps, using estimate: {:.2f}m".format(path_length))
            except Exception as e:
                path_length = len(PATH) * 0.5
                print("[PROGRESS] Error parsing timestamps: {}, using estimate: {:.2f}m".format(e, path_length))
        else:
            path_length = len(PATH) * 0.5 if len(PATH) > 0 else 1.0
            print("[PROGRESS] Using default estimate for {} commands: {:.2f}m".format(
                len(PATH), path_length))

    # Try to get ACTUAL distance from odometry (transferred from robot via SCP every 0.5s)
    try:
        with open('/home/halab2/buzz_shared/path_files/robot_odom.json', 'r') as f:
            odom_data = json.load(f)
            current_x = odom_data['x']
            current_y = odom_data['y']
            initial_x = odom_data['initial_x']
            initial_y = odom_data['initial_y']

            # Calculate actual distance traveled from odometry
            distance_traveled = math.sqrt(
                (current_x - initial_x)**2 +
                (current_y - initial_y)**2
            )

            # Estimate waypoint progress (linear interpolation based on ACTUAL path length)
            max_waypoint = len(PATH) - 1 if len(PATH) > 0 else 44
            if path_length > 0:
                waypoint_progress = (distance_traveled / path_length) * max_waypoint
                _robot_waypoint_progress = int(waypoint_progress)
                _robot_waypoint_progress = min(_robot_waypoint_progress, max_waypoint)

                # Debug output
                pct = (waypoint_progress / max_waypoint * 100) if max_waypoint > 0 else 0
                print("[PROGRESS] 🤖 ODOMETRY: progress={:.1f}/{} ({:.0f}%), distance={:.2f}m / {:.2f}m path".format(
                    waypoint_progress, max_waypoint, pct, distance_traveled, path_length))

                return _robot_waypoint_progress
    except FileNotFoundError:
        print("[PROGRESS] ⚠️  No odometry file yet (/home/halab2/buzz_shared/path_files/robot_odom.json not found)")
        print("[PROGRESS]    Waiting for buzz_cam_haptic.py on robot to transfer odometry (every 0.5s)...")
    except Exception as e:
        print("[PROGRESS] ⚠️  Error reading odometry: {}".format(e))

    # FALLBACK: If odometry unavailable, use time-based estimation
    if _last_progress_update_time == 0:
        _last_progress_update_time = time.time()
        _robot_waypoint_progress = 0
        return 0

    elapsed = time.time() - _last_progress_update_time
    distance_traveled = robot_speed * elapsed

    max_waypoint = len(PATH) - 1 if len(PATH) > 0 else 44
    if path_length > 0:
        waypoint_progress = (distance_traveled / path_length) * max_waypoint
        _robot_waypoint_progress = int(waypoint_progress)

        if int(elapsed) % 2 == 0 and elapsed > 0:
            pct = (waypoint_progress / max_waypoint * 100) if max_waypoint > 0 else 0
            print("[PROGRESS] ⏱️  TIME-BASED (fallback): progress={:.1f}/{} ({:.0f}%), distance={:.2f}m".format(
                waypoint_progress, max_waypoint, pct, distance_traveled))
        _robot_waypoint_progress = min(_robot_waypoint_progress, max_waypoint)

    return _robot_waypoint_progress

def create_scene_signature(labels_list):
    """Create a compact signature of the scene for comparison"""
    if not labels_list or labels_list == ["No objects detected!"]:
        return {"type": "empty", "objects": []}

    objects = []
    for label in labels_list:
        if "No objects detected" in label:
            continue
        # Extract class name and depth from label
        try:
            parts = label.split("|")
            class_name = parts[0].split(":")[-1].strip()
            depth_str = parts[2].split(":")[-1].strip()

            if "Invalid" not in depth_str:
                depth = float(depth_str.replace("m", ""))
                objects.append({"class": class_name, "depth": depth})
        except:
            continue

    return {"type": "objects" if objects else "empty", "objects": objects}

def scenes_are_similar(sig1, sig2, threshold=0.3):
    """Compare two scene signatures to see if they're similar enough to skip caption"""
    if sig1 is None or sig2 is None:
        return False

    # Both empty - similar
    if sig1["type"] == "empty" and sig2["type"] == "empty":
        return True

    # One empty, one not - different
    if sig1["type"] != sig2["type"]:
        return False

    # Compare objects
    objs1 = sig1["objects"]
    objs2 = sig2["objects"]

    # Different number of objects - check if difference is significant
    if abs(len(objs1) - len(objs2)) > 1:
        return False

    # Check if same classes exist with similar depths
    classes1 = {obj["class"] for obj in objs1}
    classes2 = {obj["class"] for obj in objs2}

    # If different classes, not similar
    if classes1 != classes2:
        return False

    # Check depth changes for matching classes
    for obj1 in objs1:
        matching_obj2 = next((o for o in objs2 if o["class"] == obj1["class"]), None)
        if matching_obj2:
            depth_change = abs(obj1["depth"] - matching_obj2["depth"])
            # If depth changed by more than threshold meters, scene is different
            if depth_change > threshold:
                return False

    return True


def video_init_realsense():
    # RealSense typically uses 640x480 resolution
    width = 640 * 2  # Double width for side-by-side display
    height = 480
    fps = 30 #15 or 30
    
    print(f"RealSense Camera: {640}x{480} @ {fps}fps")
    
    # Prepare video writer
    codecs = [('mp4v', '.mp4'), ('avc1', '.mp4'), ('XVID', '.avi')]
    writer = None
    for codec, ext in codecs:
        output_file = f"{'./'}realsense_record_{datetime.now().strftime('%Y%m%d_%H%M%S')}{ext}"
        VIDEO = output_file
        fourcc = cv2.VideoWriter_fourcc(*codec)
        writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
        if writer.isOpened():
            print(f"Using codec: {codec}")
            break
        writer = None
    
    if not writer:
        print("ERROR: No working codec found")
        return None, None
    
    return writer, output_file
"""
def claude(label):
    prompt = "You are given a number of seconds worth of data from three consecutive timestamps. Using this data, tell the human why you are moving a certain way and what you are avoiding. Also, mention any objects in the frames and their distance from the human. Let the person know if the given path must change due to obstacles." \
        " Keep the response concise and relevant to the human's navigation."
    system_prompt = "Responses need to be 8 words maximum and written in the point of view of a robot guide dog. The robot guide dog is helping a human to navigate around obstacles in the space. " \
    "Make sure your response helps the human understand why you are moving a certain way. Do not give specific information about the location of the objects. For example, if the depth data is 0.5m, do not say 'the object is 0.5m away'. Instead, say something like 'the object is close to you'. " \
    "Do not mind small changes in the FOV or depth data. Pay attention to and comment on larger changes in data among frame data given to you"
    # consecutive timestamp and a path giving future trajectory data for the robot.
    try:
        # First test network connectivity
        print("Testing network connectivity to Claude API...")
        import socket
        import requests

        # Test DNS resolution
        try:
            socket.gethostbyname('api.anthropic.com')
            print("✓ DNS resolution successful")
        except socket.gaierror as e:
            print(f"✗ DNS resolution failed: {e}")
            return "Network error: Cannot resolve API endpoint"

        # Test API endpoint reachability
        try:
            test_response = requests.get('https://api.anthropic.com', timeout=10)
            print("✓ API endpoint is reachable")
        except requests.exceptions.RequestException as e:
            print(f"API endpoint unreachable: {e}")
            return "Network error: Cannot reach Claude API"

        # Now try the actual API call
        print("Making Claude API request...")
        response = CLIENT.messages.create(
            #model="claude-sonnet-4-5-20250929",
            model="claude-sonnet-4-5-20250929",
            max_tokens=100,
            system=system_prompt,
            messages=[{
                "role": "user",
                "content": [
                    {"type": "text", "text": label},
                    {"type": "text", "text": prompt}
                ]
            }]
        )

        response_content = response.content[0].text
        print(f"✓ Claude API response received: {response_content}")
        return response_content

    except anthropic.APIConnectionError as e:
        error_msg = f"Network connection error: {e}"
        logging.error(error_msg)
        print(f"✗ {error_msg}")
        return "Connection error: Cannot reach Claude service"

    except anthropic.APIStatusError as e:
        error_msg = f"API error (status {e.status_code}): {e}"
        logging.error(error_msg)
        print(f"✗ {error_msg}")
        if e.status_code == 401:
            return "API key error: Invalid authentication"
        elif e.status_code == 429:
            return "API limit exceeded: Too many requests"
        else:
            return f"API error: {e.status_code}"

    except anthropic.AuthenticationError as e:
        error_msg = f"Authentication error: {e}"
        logging.error(error_msg)
        print(f"✗ {error_msg}")
        return "Authentication failed: Check API key"

    except Exception as e:
        error_msg = f"Unexpected error: {e}"
        logging.error(error_msg)
        print(f"✗ {error_msg}")
        return "Service temporarily unavailable"

"""    

def calculate_text_similarity(text1, text2):
    """Calculate similarity between two strings using character trigrams (Jaccard index)."""
    if not text1 or not text2:
        return 0.0
    t1, t2 = text1.lower().strip(), text2.lower().strip()
    if t1 == t2:
        return 1.0
    set1 = set(t1[i:i+3] for i in range(len(t1) - 2))
    set2 = set(t2[i:i+3] for i in range(len(t2) - 2))
    if not set1 or not set2:
        return 0.0
    return len(set1 & set2) / len(set1 | set2)



def emit_haptic_feedback(label, condition_mode=CONDITION):
    """
    Emit haptic signals based on DEPTH THRESHOLDS - only when robot gets close enough.
    Uses zones: cleared (>1.5m), far (1.0-1.5m), near (0.5-1.0m), close (<0.5m)

    Condition 1 (Map-based):
    - start, turn_left/right, arrive, obstacle (with spatial left/right)

    Condition 2 (Decision-based with warnings):
    - start_warning → start (sequence)
    - turn_warning → turn_left/right (sequence)
    - arrive_warning → arrive (sequence)
    - obstacle_warning → obstacle → slight_turn_left/right → cleared (sequence)
    """
    global PATH, _previous_depth_zone, _path_signal_state, _robot_waypoint_progress, _last_progress_update_time
    print("[EMIT_HAPTIC] ========== CALLED ==========", flush=True)
    print("[EMIT_HAPTIC] Label: {}".format(label[:80]), flush=True)
    print("[EMIT_HAPTIC] Condition: {}".format(condition_mode), flush=True)
    print("[EMIT_HAPTIC] Previous zone: {}".format(_previous_depth_zone), flush=True)

    # PATH is loaded once at startup via initialize_haptics()
    # No need to reload every frame - wastes I/O and clears signal state unnecessarily
    print("[EMIT_HAPTIC] PATH loaded: {} commands".format(len(PATH)))

    has_obstacle = False
    fov_left = False
    fov_right = False
    obstacle_depth = None
    current_zone = None  # Default: no object (None, not "cleared")

    # Skip obstacle processing for turn signals (no depth info)
    if "turn" in label.lower():
        print("[EMIT_HAPTIC] Turn signal detected: {}".format(label), flush=True)
        emit_haptic_signal("TURN", NAV_HAPTICS.turn_right if "right" in label.lower() else NAV_HAPTICS.turn_left, label)
        return

    # START signal removed - system runs immediately once Path.txt is created
    # (no need to wait for robot movement to begin)

    # Track waypoint progress (logs when reaching each waypoint)
    wp_idx, progress_pct, distance_remaining = track_waypoint_progress()
    if wp_idx is not None and progress_pct is not None:
        # Get current odometry for SRT logging
        try:
            with open('/home/halab2/buzz_shared/path_files/robot_odom.json', 'r') as f:
                odom_data = json.load(f)
                odom_x = odom_data.get('x', 0.0)
                odom_y = odom_data.get('y', 0.0)
                heading = math.degrees(odom_data.get('yaw', 0.0))
                if _waypoint_list and wp_idx < len(_waypoint_list):
                    wp = _waypoint_list[wp_idx]
                    write_waypoint_to_srt(wp_idx, wp[0], wp[1], odom_x, odom_y, heading, progress_pct, distance_remaining)
        except:
            pass

    # Check for predictive turn detection using waypoint heading analysis
    # This looks ahead at waypoint headings to signal turns BEFORE they start
    print("[EMIT_HAPTIC] Checking for predictive turns...", flush=True)
    print("[EMIT_HAPTIC] DEBUG: _waypoint_data_with_headings has {} waypoints".format(len(_waypoint_data_with_headings)), flush=True)
    try:
        with open('/home/halab2/buzz_shared/path_files/robot_odom.json', 'r') as f:
            odom_data = json.load(f)
            odom_x = odom_data.get('x', 0.0)
            odom_y = odom_data.get('y', 0.0)
            heading = math.degrees(odom_data.get('yaw', 0.0))
            current_pos = (odom_x, odom_y)

            # Find closest waypoint to current position
            if _waypoint_data_with_headings:
                min_dist = float('inf')
                closest_wp_idx = 0
                for idx, (wpx, wpy, _) in enumerate(_waypoint_data_with_headings):
                    dist = math.sqrt((odom_x - wpx)**2 + (odom_y - wpy)**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_wp_idx = idx

                # Look ahead for upcoming turns
                print("[EMIT_HAPTIC] DEBUG: Calling predict_upcoming_turn() at WP {}".format(closest_wp_idx), flush=True)
                upcoming_turn = predict_upcoming_turn(closest_wp_idx)
                if upcoming_turn:
                    dist_to_turn = upcoming_turn.get('distance_to_turn', 0)
                    print("[PREDICT_TURN] ✓ Upcoming turn detected: {} turn, distance {:.2f}m".format(upcoming_turn.get('turn_type'), dist_to_turn), flush=True)
                    print("[PREDICT_TURN] DEBUG: Threshold is {:.2f}m, within range: {}".format(SIGNAL_DISTANCE_THRESHOLD, dist_to_turn < SIGNAL_DISTANCE_THRESHOLD), flush=True)
                    turn_signal_emitted = emit_predictive_turn_signal(upcoming_turn, current_pos)
                    if turn_signal_emitted:
                        print("[EMIT_HAPTIC] ✓ Predictive TURN_{} signal EMITTED and logged".format(upcoming_turn['turn_type']), flush=True)
                    else:
                        print("[EMIT_HAPTIC] ⚠️  Predictive TURN_{} was NOT emitted (already signaled or distance too far)".format(upcoming_turn['turn_type']), flush=True)
                else:
                    print("[PREDICT_TURN] ✗ No upcoming turn detected at WP {}".format(closest_wp_idx), flush=True)
            else:
                print("[EMIT_HAPTIC] DEBUG: _waypoint_data_with_headings is EMPTY - cannot check for predictive turns", flush=True)
    except Exception as e:
        import traceback
        print("[EMIT_HAPTIC] ERROR in predictive turn detection: {}".format(str(e)), flush=True)
        print("[EMIT_HAPTIC] Traceback: {}".format(traceback.format_exc()), flush=True)

    # Legacy odometry-based turn detection (kept for reference, not used if predictive succeeds)
    print("[EMIT_HAPTIC] Calling detect_turn_from_odometry()...", flush=True)
    detected_turn = detect_turn_from_odometry(turn_threshold=45)
    if detected_turn == "left":
        print("[EMIT_HAPTIC] LEFT TURN detected from odometry (fallback)", flush=True)
        # Log turn to SRT with current odometry
        try:
            with open('/home/halab2/buzz_shared/path_files/robot_odom.json', 'r') as f:
                odom_data = json.load(f)
                odom_x = odom_data.get('x', 0.0)
                odom_y = odom_data.get('y', 0.0)
                heading = math.degrees(odom_data.get('yaw', 0.0))
                write_turn_to_srt("left", odom_x, odom_y, heading)
        except:
            pass
    elif detected_turn == "right":
        print("[EMIT_HAPTIC] RIGHT TURN detected, calling emit_haptic_signal()", flush=True)
        # Log turn to SRT with current odometry
        try:
            with open('/home/halab2/buzz_shared/path_files/robot_odom.json', 'r') as f:
                odom_data = json.load(f)
                odom_x = odom_data.get('x', 0.0)
                odom_y = odom_data.get('y', 0.0)
                heading = math.degrees(odom_data.get('yaw', 0.0))
                write_turn_to_srt("right", odom_x, odom_y, heading)
        except:
            pass
        # Dont determine turn from odometry, just use the pre-planned path for turn signals
        # emit_haptic_signal("TURN_RIGHT", NAV_HAPTICS.turn_right, "Real-time turn RIGHT (odometry)")
    else:
        print("[EMIT_HAPTIC] No turn detected", flush=True)

    # Check for arrival detection from odometry distance
    print("[EMIT_HAPTIC] Calling detect_arrive_from_odometry()...", flush=True)
    if detect_arrive_from_odometry():
        print("[EMIT_HAPTIC] ARRIVE detected, calling emit_haptic_signal()", flush=True)
        # Log arrival to SRT with current odometry
        try:
            with open('/home/halab2/buzz_shared/path_files/robot_odom.json', 'r') as f:
                odom_data = json.load(f)
                odom_x = odom_data.get('x', 0.0)
                odom_y = odom_data.get('y', 0.0)
                heading = math.degrees(odom_data.get('yaw', 0.0))
                write_arrive_to_srt(odom_x, odom_y, heading)
        except:
            pass
        emit_haptic_signal("ARRIVE", NAV_HAPTICS.arrive, "ARRIVE signal (destination reached via odometry)")
    else:
        print("[EMIT_HAPTIC] ARRIVE not detected", flush=True)

    # Extract depth from label: "Depth: 0.718m"
    if "No objects detected" not in label:
        has_obstacle = True
        import re
        depth_match = re.search(r'Depth:\s*([\d.]+)', label)
        if depth_match:
            try:
                obstacle_depth = float(depth_match.group(1))
                print("[EMIT_HAPTIC] Depth: {:.3f}m (raw float)".format(obstacle_depth), flush=True)
                print("[EMIT_HAPTIC] Thresholds: CLOSE={:.2f}, NEAR={:.2f}, FAR={:.2f}".format(CLOSE_THRESHOLD, NEAR_THRESHOLD, FAR_THRESHOLD), flush=True)

                # Classify into depth zones based on configurable thresholds
                if obstacle_depth < CLOSE_THRESHOLD:
                    current_zone = "close"
                    debug_msg = "Zone: CLOSE (depth {:.3f}m < {:.2f}m)".format(obstacle_depth, CLOSE_THRESHOLD)
                    print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
                    write_emit_debug_to_srt(debug_msg)
                elif obstacle_depth < NEAR_THRESHOLD:
                    current_zone = "near"
                    debug_msg = "Zone: NEAR (depth {:.3f}m < {:.2f}m)".format(obstacle_depth, NEAR_THRESHOLD)
                    print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
                    write_emit_debug_to_srt(debug_msg)
                elif obstacle_depth < FAR_THRESHOLD:
                    current_zone = "far"
                    debug_msg = "Zone: FAR (depth {:.3f}m < {:.2f}m)".format(obstacle_depth, FAR_THRESHOLD)
                    print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
                    write_emit_debug_to_srt(debug_msg)
                else:
                    current_zone = None  # Beyond far threshold - treat as no obstacle
                    debug_msg = "Zone: CLEARED (depth {:.3f}m >= {:.2f}m)".format(obstacle_depth, FAR_THRESHOLD)
                    print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
                    write_emit_debug_to_srt(debug_msg)
            except:
                current_zone = "far"  # Default if parse fails

        # Log detection with zone to SRT file
        if current_zone and has_obstacle:
            write_detection_to_srt(label, zone=current_zone)

        # Determine obstacle position (left/right)
        if "negative" in label.lower() or "left" in label.lower():
            fov_left = True
        if "positive" in label.lower() or "right" in label.lower():
            fov_right = True

    # === CONDITION 1: Map-based (simple individual signals) ===
    if condition_mode == 1:
        # Only signal if obstacle is within CLOSE threshold (tight/sensitive)
        if has_obstacle and current_zone == "close" and _previous_depth_zone != "close":
            debug_msg = "C1 DECISION: SIGNAL (obstacle in CLOSE zone, depth < {:.2f}m)".format(CLOSE_THRESHOLD)
            print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
            write_emit_debug_to_srt(debug_msg)
            try:
                NAV_HAPTICS.obstacle()
                print("[EMIT_HAPTIC] C1: obstacle() SUCCESS - SIGNAL FIRED")
            except Exception as e:
                print("[EMIT_HAPTIC] C1 obstacle FAILED: {}".format(str(e)))
        
        else:
            if has_obstacle:
                debug_msg = "C1 DECISION: NO SIGNAL (obstacle depth {:.3f}m >= {:.2f}m)".format(
                    obstacle_depth, CLOSE_THRESHOLD)
            else:
                debug_msg = "C1 DECISION: NO SIGNAL (no obstacle detected)"
            print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
            write_emit_debug_to_srt(debug_msg)
        _previous_depth_zone = current_zone
        # NOTE: Turns and arrival are now detected from REAL ODOMETRY
        # (not from pre-planned PATH waypoints)

    # === CONDITION 2: Decision-based (grouped sequential signals) ===
    if condition_mode == 2:
        debug_msg = "C2 CHECK: {} → {}".format(_previous_depth_zone, current_zone)
        print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
        write_emit_debug_to_srt(debug_msg)
        # Only emit when CROSSING a depth threshold (zone changes)
        if has_obstacle and current_zone == "close" and _previous_depth_zone != "close":
            debug_msg = "C2 DECISION: SIGNAL (obstacle in CLOSE zone, depth < {:.2f}m)".format(CLOSE_THRESHOLD)
            print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
            write_emit_debug_to_srt(debug_msg)
            try:
                signals = ["obstacle"]
                if fov_left:
                    signals.append("slight_turn_left")
                elif fov_right:
                    signals.append("slight_turn_right")
                # signals.append("cleared")
                NAV_HAPTICS.emit_sequence(signals)
                print("[EMIT_HAPTIC] C2: Obstacle sequence SUCCESS")
            except Exception as e:
                print("[EMIT_HAPTIC] C2: Obstacle sequence FAILED: {}".format(str(e)))
        else:
            if has_obstacle:
                debug_msg = "C2 DECISION: NO SIGNAL (obstacle depth {:.3f}m >= {:.2f}m)".format(
                    obstacle_depth, FAR_THRESHOLD)
                print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
                write_emit_debug_to_srt(debug_msg)
            else:
                debug_msg = "C2 DECISION: NO SIGNAL (no obstacle detected)"
                print("[EMIT_HAPTIC] {}".format(debug_msg), flush=True)
                write_emit_debug_to_srt(debug_msg)

        # NOTE: Turns and arrival are now detected from REAL ODOMETRY
        # (not from pre-planned PATH waypoints)

        # Update previous zone for next frame (enables transition detection)
        _previous_depth_zone = current_zone

def sound_check_thread(label, condition_mode=2):
    """Emit haptic feedback. Signals automatically logged to UI via NavHapticsWithUI wrapper."""
    print("[HAPTIC] Emitting signal (condition {}): {}...".format(condition_mode, label[:50]), flush=True)

    # Store detection label in thread-local storage for SRT logging
    _current_detection_label.label = label

    try:
        print("[HAPTIC] Calling emit_haptic_feedback()...", flush=True)
        emit_haptic_feedback(label, condition_mode=condition_mode)
        print("[HAPTIC] emit_haptic_feedback() returned", flush=True)
        print("[HAPTIC] Signal sent to motors and logged to UI")
        # Verify JSON was written
        import os
        json_log = "/tmp/haptic_motor_log.json"
        if os.path.exists(json_log):
            with open(json_log) as f:
                lines = f.readlines()
            print("[HAPTIC] JSON log now has {} events".format(len(lines)))
    except Exception as e:
        logging.error("[HAPTIC] Error: {}".format(e))
        import traceback
        traceback.print_exc()


# Audio thread removed - haptic feedback only

def frame_thread_worker():
    """Process frames and emit haptic feedback based on detections only."""
    global processing_active
    time_step = 0
    print("Frame thread worker started.")
    while processing_active:
        try:
            frame_data = FRAME_Q.get(timeout=1.0)
            print(f"[Frame Worker] Got frame from queue")

            # Save frame
            save_frame(SCREENSHOT_DIR, frame_data['combined'], time_step, frame_data['labels'])

            # Get detection labels and emit haptic feedback
            labels = frame_data.get('labels', [])
            if labels:
                for label in labels:
                    emit_haptic_feedback(label, condition_mode=CONDITION)

            time_step += 1

        except Empty:
            continue

        except Exception as e:
            if type(e).__name__ != 'Empty':
                import traceback
                print(f"\n{'='*60}")
                print(f"ERROR in frame_processing_thread:")
                print(f"Exception type: {type(e).__name__}")
                print(f"Exception message: {str(e)}")
                print(f"Full traceback:")
                traceback.print_exc()
                print(f"{'='*60}\n")
                continue
    print("Frame thread worker stopped.")

def setup_logging_and_output(output_dir):
    """Initialize logging and create output directory"""
    os.makedirs(SCREENSHOT_DIR, exist_ok=True)
    logging.basicConfig(
        filename= os.path.join(BASE_OUTPUT_DIR, "test.txt"), 
        filemode="w", 
        level=logging.INFO, 
        format="%(asctime)s - %(message)s", 
        datefmt="%H:%M:%S"
    )
    return SCREENSHOT_DIR

def initialize_realsense_camera():
    """Initialize and configure Intel RealSense D435"""
    print("Setting up Intel RealSense D435...", flush=True)

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    print("[INIT] Starting pipeline...", flush=True)
    profile = pipeline.start(config)
    print("[INIT] Pipeline started successfully", flush=True)

    # Wait for camera to warm up (increased from 0.5s to 2s for reliability)
    print("[INIT] Camera warming up...", flush=True)
    for i in range(4):
        time.sleep(0.5)
        print("[INIT]   ...warming ({}s)".format((i+1)*0.5), flush=True)
    print("[INIT] Camera warm-up done", flush=True)

    # Try to grab initial frame to verify camera is streaming
    print("[INIT] Testing initial frame grab...", flush=True)
    try:
        # Increased timeout from 2000ms to 5000ms for more reliable initialization
        test_frames = pipeline.wait_for_frames(timeout_ms=5000)
        print("[INIT] Initial frame successful!", flush=True)
    except Exception as e:
        print("[INIT] WARNING: Initial frame failed: {}".format(e), flush=True)
        print("[INIT] Retrying with longer timeout...", flush=True)
        try:
            # One more attempt with even longer timeout
            test_frames = pipeline.wait_for_frames(timeout_ms=10000)
            print("[INIT] Initial frame successful (retry)!", flush=True)
        except Exception as e2:
            print("[INIT] FATAL: Frame grab failed even with extended timeout: {}".format(e2), flush=True)
            raise

    # Get camera intrinsics
    color_profile = profile.get_stream(rs.stream.color)
    intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
    camera_intrinsics = (intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    print("[INIT] Camera intrinsics: fx={}, fy={}".format(intrinsics.fx, intrinsics.fy), flush=True)

    # Align depth to color
    align_to = rs.stream.color
    align = rs.align(align_to)
    print("[INIT] Alignment configured", flush=True)

    return pipeline, align, camera_intrinsics

def prepare_realsense_frames(frames, align):
    """Process RealSense frames to get aligned color and depth images"""
    # Align the depth frame to color frame
    aligned_frames = align.process(frames)
    
    # Get aligned frames
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    if not depth_frame or not color_frame:
        return None, None, None
    
    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    
    # Process depth for visualization
    depth_colored = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.03), 
        cv2.COLORMAP_JET
    )
    
    return color_image, depth_colored, depth_frame

def process_detections_realsense(results, color_image, depth_frame, annotated_rgb, annotated_depth, camera_intrinsics):
    """Process YOLO detections with RealSense depth data

    Returns:
        Tuple of (in_fov_labels, all_detections) where:
        - in_fov_labels: labels for objects that passed FOV filter (for haptic)
        - all_detections: all chair detections with FOV status (for SRT logging)
    """
    fx, fy, px, py = camera_intrinsics
    in_fov_labels = []
    all_detections = []  # Track all chairs with FOV status

    for result in results:
        if len(result.boxes) > 0:
            for box_idx in range(len(result.boxes)):
                x1, y1, x2, y2 = map(int, result.boxes.xyxy[box_idx].cpu().numpy())

                # Get detection information
                cls_id = int(result.boxes.cls[box_idx]) #detection.boxes
                cls_name = result.names[cls_id] #results[0].names
                if cls_name in IGNORED_CLASSES:
                    continue

                # Filter 1: Only process chairs for haptic signals
                if cls_name.lower() != HAPTIC_TARGET_CLASS.lower():
                    continue

                confidence = result.boxes.conf[box_idx] * 100
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                # Get depth value (convert from mm to meters)
                depth_value = depth_frame.get_distance(center_x, center_y) if depth_frame else 0

                # Angles
                theta_x = math.degrees(math.atan2((center_x - px), fx))
                theta_y = math.degrees(math.atan2((center_y - py), fy))

                # Create label with depth info
                if depth_value == 0 or math.isinf(depth_value):
                    label = f"Class: {cls_name} | Confidence: {confidence:.2f}% | Depth: Invalid | Horizontal FOV: {theta_x:.2f}deg"
                else:
                    label = f"Class: {cls_name} | Confidence: {confidence:.2f}% | Depth: {depth_value:.3f}m | Horizontal FOV: {theta_x:.2f}deg"

                # Check FOV threshold
                in_fov = abs(theta_x) <= FOV_THRESHOLD

                if in_fov:
                    in_fov_labels.append(label)
                    print(f"[DETECTION] {cls_name} IN_FOV: θ_x={theta_x:.2f}° (within ±{FOV_THRESHOLD}°)", flush=True)
                else:
                    print(f"[DETECTION] {cls_name} OUT_OF_FOV: θ_x={theta_x:.2f}° (outside ±{FOV_THRESHOLD}°)", flush=True)

                # Track all detections with FOV status for SRT logging
                detection_info = {
                    'label': label,
                    'in_fov': in_fov,
                    'theta_x': theta_x,
                    'confidence': confidence,
                    'depth': depth_value
                }
                all_detections.append(detection_info)

                # Draw on both images
                for img in [annotated_rgb, annotated_depth]:
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Return both filtered labels (for haptic) and all detections (for logging)
    haptic_labels = in_fov_labels if in_fov_labels else ["No objects detected!"]
    return haptic_labels, all_detections

def save_frame(output_dir, combined, time_step, labels):
    """Save frame and log information"""
    frame_path = os.path.join(output_dir, f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")
    cv2.imwrite(frame_path, combined)
    print(f"Saved: {frame_path}")
    return labels

def add_captions(frame, text):
    if not text:
        return frame
    # Add Captions
    print(f"Adding caption: {text}")
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (50, 50)
    font_scale = 0.7
    font_color = (0, 255, 0)
    thickness = 2
    line_type = cv2.LINE_AA

    (text_width, text_height) = cv2.getTextSize(text, font, font_scale, thickness)
    y_pos = frame.shape[0] - 10

    cv2.rectangle(frame, (0, y_pos - text_height - 10), (frame.shape[1], y_pos + 10), (0, 0, 0), -1)
    cv2.putText(frame, text, (10, y_pos - 5), font, font_scale, font_color, thickness)

    return frame

def run_with_depth(pipeline, align, model, output_dir, camera_intrinsics, writer):
    frame_num = 0
    captured_frames = deque(maxlen=3)  # Keep last 3 frames for processing
    processing_stop = [False]  # Wrap in list for closure mutability

    def process_frames_background():
        """Background thread: Process captured frames (YOLO, file I/O, display)"""
        nonlocal frame_num
        while not processing_stop[0] or captured_frames:
            if not captured_frames:
                time.sleep(0.001)  # Tiny sleep to avoid busy-wait
                continue

            try:
                frame_data = captured_frames.popleft()
                color_image = frame_data['color']
                depth_colored = frame_data['depth']
                depth_frame = frame_data['depth_frame']
                start_time = frame_data['timestamp']

                # Run object detection (slow operation)
                results = model(source=color_image, conf=0.69)
                annotated_rgb = color_image.copy()
                annotated_depth = depth_colored.copy()

                # Process detections (returns both in-FOV labels for haptic and all detections for logging)
                haptic_labels, all_detections = process_detections_realsense(results, color_image, depth_frame,
                                                    annotated_rgb, annotated_depth,
                                                    camera_intrinsics)

                print("[DEBUG] Haptic labels: {} | All detections: {}".format(len(haptic_labels), len(all_detections)), flush=True)

                # Log ALL detections (both in-FOV and out-of-FOV) to SRT file with FOV status
                for detection in all_detections:
                    fov_status = "IN_FOV" if detection['in_fov'] else "OUT_OF_FOV"
                    detection_label = f"{detection['label']} | FOV_STATUS: {fov_status}"
                    print("[FRAME] Logging detection to SRT: {}".format(detection_label[:70]), flush=True)
                    write_detection_to_srt(detection_label)

                # Emit haptic feedback (always, even if no detections - arrival must run regardless)
                label_str = " | ".join(haptic_labels)
                print("[FRAME] Spawning haptic thread for: {} (condition {})".format(label_str[:60], CONDITION), flush=True)
                haptic_thread = threading.Thread(
                    target=sound_check_thread,
                    args=(label_str, CONDITION),
                    daemon=True
                )
                haptic_thread.start()

                # Get expected dimensions from writer
                expected_width = 640 * 2
                expected_height = 480

                # Display combined image
                combined = np.hstack((annotated_rgb, annotated_depth))
                if combined.shape[1] != expected_width or combined.shape[0] != expected_height:
                    combined = cv2.resize(combined, (expected_width, expected_height))

                if combined.shape[2] == 3:
                    frame_to_write = cv2.cvtColor(combined, cv2.COLOR_RGB2BGR)
                else:
                    frame_to_write = combined

                cv2.imshow("RealSense Camera (Color + Depth)", frame_to_write)
                writer.write(frame_to_write)

                # Log every frame's YOLO detections to file (all detections with FOV status)
                with open(detections_filename, 'a', encoding='utf-8') as f:
                    f.write(f"Frame {frame_num} @ {start_time}\n")
                    for detection in all_detections:
                        fov_status = "IN_FOV" if detection['in_fov'] else "OUT_OF_FOV"
                        f.write(f"  {detection['label']} | FOV_STATUS: {fov_status}\n")
                    if not all_detections:
                        f.write(f"  No objects detected!\n")
                    f.write("\n")

                frame_num += 1

                if chr(cv2.waitKey(1) & 0xFF) == 'q':
                    processing_stop[0] = True
            except Exception as e:
                print(f"[ERROR] Frame processing error: {e}")

    # Start background processing thread
    processor_thread = threading.Thread(target=process_frames_background, daemon=True)
    processor_thread.start()

    try:
        # Main thread: Just capture frames, don't process
        print("[MAIN] Starting frame capture loop...", flush=True)
        while True:
            try:
                print("[MAIN] Waiting for frame...", flush=True)
                frames = pipeline.wait_for_frames(timeout_ms=3000)
                print("[MAIN] Frame received", flush=True)

                # Prepare frames (minimal processing)
                color_image, depth_colored, depth_frame = prepare_realsense_frames(frames, align)
                if color_image is None or depth_colored is None:
                    print("[MAIN] Frame preparation failed, skipping", flush=True)
                    continue

                # Queue for background processing
                if len(captured_frames) >= 3:
                    print("[WARN] Frame queue full (maxlen=3), dropping oldest frame", flush=True)
                captured_frames.append({
                    'color': color_image.copy(),
                    'depth': depth_colored.copy(),
                    'depth_frame': depth_frame,
                    'timestamp': datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                })

                if processing_stop[0]:
                    break

            except Exception as e:
                print("[ERROR] Frame capture error: {}".format(e), flush=True)
                import traceback
                traceback.print_exc()
                break
    finally:
        processing_stop[0] = True
        processor_thread.join(timeout=2)
        pipeline.stop()
        writer.release()
        cv2.destroyAllWindows()

def main():
    global processing_active, camera_active
    for directory in [BASE_OUTPUT_DIR, OUT_DIR, IN_DIR, AUDIO_DIR, VIDEO_DIR, SCREENSHOT_DIR, TIMING_DIR, DETECTIONS_DIR, HAPTIC_DIR]:
        os.makedirs(directory, exist_ok=True)

    # Remove old Path.txt and FULL_PATH.txt to avoid stale data
    files = ['/home/halab2/buzz_shared/path_files/FULL_PATH.txt', '/home/halab2/buzz_shared/path_files/Path.txt']
    for file in files:
        if os.path.exists(file):
            os.remove(file)

    if not initialize_haptics():
        raise RuntimeError("Haptic initialization failed. Check motor connections and permissions.")
    
    # Initialize SRT file for haptic signals
    init_haptic_srt()

    # Set SRT callback for signal logging in haptic_ui_bridge
    from haptic_ui_bridge import set_srt_callback
    set_srt_callback(write_signal_to_srt)
    print("[HAPTIC] SRT callback registered", flush=True)

    mode_label = "(PSEUDO DATA MODE)" if USE_PSEUDO_DATA else ""
    print("\n" + "=" * 50)
    print(f"🎥 ROBOT CAMERA + HAPTIC NAVIGATION {mode_label}".strip())
    print("=" * 50)
    if USE_PSEUDO_DATA:
        print("Simulated camera data + pseudo path/odometry")
    else:
        print("Real RealSense camera + real path/odometry")
    print("Detection-based haptic feedback (LLM removed)")
    print("=" * 50 + "\n")

    # Handle path generation based on mode
    if USE_PSEUDO_DATA:
        print("[STARTUP] 📝 Generating pseudo path data...")
        generate_pseudo_path_data()
        print("[STARTUP] 📝 Deriving Path.txt from FULL_PATH.txt...")
        generate_path_from_full_path()
        print("[STARTUP] ✓ Pseudo data generated")

        # Start odometry simulation in background (for waypoint tracking)
        print("[STARTUP] 🤖 Starting odometry simulation thread...")
        odom_thread = threading.Thread(
            target=simulate_pseudo_odometry,
            args=(_waypoint_list, 15.0, 0.5),  # 15 second simulation, update every 0.5s
            daemon=True
        )
        odom_thread.start()
        print("[STARTUP] ✓ Odometry simulation started (15s duration)", flush=True)
    else:
        # Live mode: Path.txt will be generated from FULL_PATH.txt by initialize_haptics()
        # No need to delete it here - the generation logic will handle it
        print("[STARTUP] Live mode: Path.txt will be generated from FULL_PATH.txt")

    vid_output_dir = setup_logging_and_output(SCREENSHOT_DIR)
    # Clean Folder
    for filename in os.listdir(vid_output_dir):
        if filename.endswith(".jpg"):
            os.remove(os.path.join(vid_output_dir, filename))

    print("Loading YOLO model...")
    model = YOLO('yolov8n.pt')

    print("Starting frame processing thread...")
    process_thread = threading.Thread(target=frame_thread_worker, daemon=True)
    process_thread.start()

    # Initialize RealSense with error handling
    pipeline = None
    align = None
    intrinsics = None
    writer = None

    try:
        if USE_PSEUDO_DATA:
            print("[PSEUDO] Pseudo data mode - monitoring odometry for signals")
            print("[PSEUDO] Haptic signals will fire: TURN (from heading), ARRIVE (from distance)")
            print("[PSEUDO] To exit, press Ctrl+C")

            # Reset turn detection state for new simulation run
            global _initial_heading, _previous_heading, _last_turn_emission_time
            _initial_heading = None
            _previous_heading = None
            _last_turn_emission_time = 0

            # Setup SRT logging for simulation
            sim_srt_file = os.path.join(HAPTIC_DIR, "sim_{}.srt".format(datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
            sim_start_time = time.time()
            sim_entry_counter = 1

            def write_sim_event(event_type, description):
                """Write simulation event to SRT file"""
                nonlocal sim_entry_counter
                elapsed = time.time() - sim_start_time
                hours = int(elapsed // 3600)
                minutes = int((elapsed % 3600) // 60)
                seconds = int(elapsed % 60)
                millis = int((elapsed % 1) * 1000)
                timestamp = "{:02d}:{:02d}:{:02d},{:03d}".format(hours, minutes, seconds, millis)

                entry = "{}\n{} --> {}\n[{}] {}\n\n".format(
                    sim_entry_counter, timestamp, timestamp, event_type, description)
                try:
                    with open(sim_srt_file, 'a') as f:
                        f.write(entry)
                except Exception as e:
                    print("[PSEUDO] Error writing SRT: {}".format(e))
                sim_entry_counter += 1

            # Write simulation start
            write_sim_event("SIM_START", "Pseudo simulation started")
            write_sim_event("CONFIG", "Condition: {}, Backend: {}, Mode: PSEUDO".format(CONDITION, BACKEND))
            write_sim_event("PATH", "FULL_PATH: 50 waypoints, Expected distance: 27.979m")
            write_sim_event("SEGMENTS", "Segment 1: FORWARD (0m→10m, 0°) | Segment 2: TURN_LEFT (45°) | Segment 3: FORWARD (10m, 45°) | Segment 4: TURN_LEFT (45°→90°) | Segment 5: FORWARD (10m, 90°) | Signal: ARRIVE")
            print("[PSEUDO] Logging to: {}".format(sim_srt_file), flush=True)

            # Monitor odometry and emit signals for pseudo mode
            last_arrived = False
            last_waypoint_logged = -1

            while processing_active:
                try:
                    # Track waypoint progress and log it
                    wp_idx, progress_pct, distance_remaining = track_waypoint_progress()
                    if wp_idx is not None and wp_idx > last_waypoint_logged:
                        last_waypoint_logged = wp_idx
                        if wp_idx < len(_waypoint_list):
                            wp = _waypoint_list[wp_idx]
                            # Read current odometry
                            try:
                                with open('/home/halab2/buzz_shared/path_files/robot_odom.json', 'r') as f:
                                    odom_data = json.load(f)
                                    odom_x = odom_data.get('x', 0.0)
                                    odom_y = odom_data.get('y', 0.0)
                                    heading = math.degrees(odom_data.get('yaw', 0.0))
                            except:
                                odom_x, odom_y, heading = 0.0, 0.0, 0.0

                            write_sim_event("WAYPOINT",
                                "WP {}/{} | Target: ({:.3f}, {:.3f}) | Odom: ({:.3f}, {:.3f}) | Heading: {:.1f}° | {:.1f}% | {:.1f}m remaining".format(
                                    wp_idx, len(_waypoint_list)-1, wp[0], wp[1], odom_x, odom_y, heading, progress_pct, distance_remaining))

                    # Helper to get current odometry
                    def get_current_odom():
                        try:
                            with open('/home/halab2/buzz_shared/path_files/robot_odom.json', 'r') as f:
                                odom_data = json.load(f)
                                ox = odom_data.get('x', 0.0)
                                oy = odom_data.get('y', 0.0)
                                h = math.degrees(odom_data.get('yaw', 0.0))
                                return ox, oy, h
                        except:
                            return 0.0, 0.0, 0.0

                    # Check for TURN signal using predictive heading-based approach
                    ox, oy, h = get_current_odom()
                    current_pos = (ox, oy)

                    # Find closest waypoint to current position
                    if _waypoint_data_with_headings:
                        min_dist = float('inf')
                        closest_wp_idx = 0
                        for idx, (wpx, wpy, _) in enumerate(_waypoint_data_with_headings):
                            dist = math.sqrt((ox - wpx)**2 + (oy - wpy)**2)
                            if dist < min_dist:
                                min_dist = dist
                                closest_wp_idx = idx

                        # Look ahead for upcoming turns
                        upcoming_turn = predict_upcoming_turn(closest_wp_idx)
                        if upcoming_turn:
                            turn_signal_emitted = emit_predictive_turn_signal(upcoming_turn, current_pos)
                            if turn_signal_emitted:
                                turn_type = upcoming_turn['turn_type']
                                write_sim_event("SIGNAL", "TURN_{} predicted | Heading: {} → {} | {:.2f}m away".format(
                                    turn_type, upcoming_turn['current_direction'],
                                    upcoming_turn['next_direction'], upcoming_turn['distance_to_turn']))
                                write_sim_event("HAPTIC", "TURN_{} signal emitted".format(turn_type))

                    # Check for ARRIVE signal
                    if detect_arrive_from_odometry() and not last_arrived:
                        last_arrived = True
                        ox, oy, h = get_current_odom()
                        print("[PSEUDO] ✓ ARRIVE signal detected and emitted!")
                        write_sim_event("SIGNAL", "ARRIVE detected | Odom: ({:.3f}, {:.3f}) | Heading: {:.1f}°".format(ox, oy, h))
                        emit_haptic_signal("ARRIVE", NAV_HAPTICS.arrive, "Destination reached")
                        write_sim_event("HAPTIC", "ARRIVE signal emitted")

                    time.sleep(0.1)  # Check every 100ms
                except Exception as e:
                    print("[PSEUDO] Error monitoring odometry: {}".format(e))
                    write_sim_event("ERROR", str(e))
                    time.sleep(0.5)
                    break

            # Write simulation end
            write_sim_event("SIM_END", "Pseudo simulation completed")
            print("[PSEUDO] Simulation logged to: {}".format(sim_srt_file), flush=True)
        else:
            print("Initializing RealSense camera...")
            pipeline, align, intrinsics = initialize_realsense_camera()

            # Initialize video writer
            print("Initializing video writer...")
            writer, video_filename = video_init_realsense()

            print("Starting processing with RealSense...")
            run_with_depth(pipeline, align, model, vid_output_dir, intrinsics, writer)
    except RuntimeError as e:
        print("[ERROR] RealSense initialization failed: {}".format(e), flush=True)
        print("[ERROR] The camera may be disconnected or in use by another process", flush=True)

        # Cleanup
        if pipeline:
            try:
                pipeline.stop()
            except:
                pass
        if writer:
            try:
                writer.release()
            except:
                pass

        print("[ERROR] Unable to start camera. Exiting.", flush=True)
    except Exception as e:
        print("[ERROR] Unexpected error: {}".format(e), flush=True)
        import traceback
        traceback.print_exc()

        # Cleanup
        if pipeline:
            try:
                pipeline.stop()
            except:
                pass
        if writer:
            try:
                writer.release()
            except:
                pass
    finally:
        # Cleanup
        processing_active = False
        camera_active = False
        process_thread.join(timeout=3.0)

        if process_thread.is_alive():
            print("Warning: Process thread did not terminate cleanly.")

        print("\n" + "=" * 40)
        print("✅ Camera + Haptic Navigation Stopped")
        print("=" * 40)

if __name__ == "__main__":
    main()