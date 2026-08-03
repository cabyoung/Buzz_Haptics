#!/usr/bin/env python3
"""
haptic_ble.py — synchronous BLE transport for the haptic belt.

Bridges blocking, thread-based callers (haptics_nav.py's demo thread) to the
nRF5340 firmware in src/, which is a BLE *peripheral* speaking the binary
protocol in src/bluetooth_peripheral.h. bleak is async-only, so this module
owns an asyncio loop on a daemon thread and exposes plain blocking methods.

Two translations happen here, because the belt script and the firmware do not
share a vocabulary:

  belt index -> motor id   The script numbers motors 0-8; the firmware numbers
                           them 1-10 (schematic labels DRV_1..DRV_10) and
                           treats id 0 as "default motor". Offset by +1.

  effect     -> pulse      The script speaks DRV2605 waveform-library IDs. The
                           DRV8837 H-bridges on this board have no waveform
                           ROM, so each effect becomes a (duration, duty)
                           one-shot PULSE the firmware times itself.

Requires: pip install bleak

Standalone smoke test:
    python haptic_ble.py            # sweep every belt motor, left to right
"""
import asyncio
import struct
import threading

from bleak import BleakClient, BleakScanner

# Must match src/bluetooth_peripheral.h
HAPTIC_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
STATUS_CHAR_UUID    = "12345678-1234-5678-1234-56789abcdef1"
CONTROL_CHAR_UUID   = "12345678-1234-5678-1234-56789abcdef2"

DEFAULT_NAME = "HapticMotor"

OP_OFF     = 0x00
OP_PULSE   = 0x04
OP_SETFREQ = 0x05

# Belt index 0-8 maps onto firmware motor ids 1-9 (DRV_10 is left unused).
MOTOR_ID_OFFSET = 1
BELT_SIZE       = 9

# PWM carrier pushed to the device on connect. 25 kHz keeps the H-bridge
# switching above the audible band so the belt is silent between pulses.
PWM_CARRIER_HZ = 25000


def belt_to_motor_id(index: int) -> int:
    """Belt position 0-8 -> firmware motor id 1-9."""
    return index + MOTOR_ID_OFFSET


# ── command encoders (little-endian, byte 0 = opcode) ─────────────────────────

def enc_pulse(on_ms: int, duty: int, motor_id: int) -> bytes:
    """PULSE: [u16 on_ms][u8 duty][u8 motor] — one-shot, timed by the device."""
    return struct.pack("<BHBB", OP_PULSE, _u16(on_ms), _duty(duty), motor_id & 0xFF)


def enc_off(motor_id: int) -> bytes:
    """OFF: [u8 motor] — stop that motor and cancel any running pattern."""
    return struct.pack("<BB", OP_OFF, motor_id & 0xFF)


def enc_freq(freq_hz: int) -> bytes:
    """SET_FREQUENCY: [u16 freq_hz] — global, applies to every motor."""
    return struct.pack("<BH", OP_SETFREQ, _u16(freq_hz))


def _duty(v) -> int:
    return max(0, min(100, int(v)))


def _u16(v) -> int:
    return max(0, min(0xFFFF, int(v)))


# ── link ──────────────────────────────────────────────────────────────────────

class BleHapticLink:
    """Blocking wrapper around a bleak connection to the haptic peripheral.

    The asyncio loop lives on a daemon thread; every public method marshals its
    coroutine onto that loop and waits for the result, so callers can stay
    fully synchronous.
    """

    def __init__(self, name=DEFAULT_NAME, address=None, scan_timeout=15.0,
                 on_status=None):
        self._name         = name
        self._address      = address
        self._scan_timeout = scan_timeout
        self._on_status    = on_status

        self._client = None
        self._loop   = None
        self._thread = None
        # Serializes writes so a group buzz is not interleaved with a sweep.
        self._write_lock = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def connect(self):
        """Scan, connect, subscribe to status. Raises on failure."""
        self._loop   = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        try:
            self._run(self._connect())
        except Exception:
            self._shutdown_loop()
            raise
        return self

    def close(self):
        """Stop every motor, disconnect, and tear the loop down."""
        if self._loop is None:
            return
        try:
            self._run(self._disconnect(), timeout=10.0)
        except Exception:
            pass
        self._shutdown_loop()

    @property
    def connected(self) -> bool:
        return self._client is not None and self._client.is_connected

    # ── commands ──────────────────────────────────────────────────────────────

    def pulse(self, index: int, on_ms: int, duty: int):
        """One-shot pulse on a single belt position. Returns immediately —
        the device times the pulse itself."""
        self._run(self._write(enc_pulse(on_ms, duty, belt_to_motor_id(index))))

    def pulse_many(self, indices, on_ms: int, duty: int):
        """Pulse several belt positions as close to simultaneously as the link
        allows: the writes are issued back-to-back inside one loop task, with
        no Python-side await between them."""
        payloads = [enc_pulse(on_ms, duty, belt_to_motor_id(i)) for i in indices]
        self._run(self._write_burst(payloads))

    def off(self, index=None):
        """Stop one belt position, or every one of them."""
        targets = range(BELT_SIZE) if index is None else [index]
        payloads = [enc_off(belt_to_motor_id(i)) for i in targets]
        self._run(self._write_burst(payloads))

    def set_frequency(self, freq_hz: int):
        """Set the PWM carrier for all motors."""
        self._run(self._write(enc_freq(freq_hz)))

    # ── internals ─────────────────────────────────────────────────────────────

    def _run(self, coro, timeout=30.0):
        if self._loop is None:
            raise RuntimeError("link not connected")
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout)

    def _shutdown_loop(self):
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)
        self._loop.close()
        self._loop = self._thread = self._client = None

    async def _connect(self):
        self._write_lock = asyncio.Lock()

        if self._address:
            print(f"[BLE] Looking for address {self._address} ...")
            device = await BleakScanner.find_device_by_address(
                self._address, timeout=self._scan_timeout)
        else:
            print(f"[BLE] Looking for '{self._name}' ...")
            device = await BleakScanner.find_device_by_name(
                self._name, timeout=self._scan_timeout)

        if device is None:
            raise RuntimeError(
                f"haptic device not found "
                f"({self._address or self._name}) — is it powered and advertising?")

        print(f"[BLE] Connecting to {device.address} ...")
        client = BleakClient(device)
        await client.connect()
        self._client = client
        print("[BLE] Connected")

        # Status notifications are optional; the belt works without them.
        try:
            await client.start_notify(STATUS_CHAR_UUID, self._status_handler)
        except Exception as exc:
            print(f"[BLE] Status notify unavailable: {exc}")

        await self._write(enc_freq(PWM_CARRIER_HZ))

    async def _disconnect(self):
        if self._client is None:
            return
        if self._client.is_connected:
            try:
                for i in range(BELT_SIZE):
                    await self._raw_write(enc_off(belt_to_motor_id(i)))
            except Exception:
                pass
            await self._client.disconnect()
        print("[BLE] Disconnected")

    def _status_handler(self, _sender, data):
        if data and self._on_status:
            self._on_status(data[0])

    async def _write(self, payload: bytes):
        async with self._write_lock:
            await self._raw_write(payload)

    async def _write_burst(self, payloads):
        async with self._write_lock:
            for payload in payloads:
                await self._raw_write(payload)

    async def _raw_write(self, payload: bytes):
        if self._client is None or not self._client.is_connected:
            raise RuntimeError("haptic device disconnected")
        # Write-without-response: no ATT round trip, so a sweep stays crisp.
        await self._client.write_gatt_char(CONTROL_CHAR_UUID, payload, response=False)


# ── smoke test ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    import time

    p = argparse.ArgumentParser(description="Haptic belt BLE smoke test")
    p.add_argument("--name", default=DEFAULT_NAME, help="advertised device name")
    p.add_argument("--address", default=None, help="BLE address (overrides --name)")
    args = p.parse_args()

    link = BleHapticLink(name=args.name, address=args.address,
                         on_status=lambda d: print(f"[STATUS] duty={d}%")).connect()
    try:
        print("Sweeping belt 0 -> 8 ...")
        for i in range(BELT_SIZE):
            print(f"  motor {i} (firmware id {belt_to_motor_id(i)})")
            link.pulse(i, 200, 100)
            time.sleep(0.35)

        print("Center group 3-4-5 together ...")
        link.pulse_many([3, 4, 5], 500, 100)
        time.sleep(1.0)
    finally:
        link.close()
