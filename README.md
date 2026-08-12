# Buzz_Haptics

Haptic navigation encoding for a guide-dog robot.

| File | Role |
| --- | --- |
| [`nav_signals.py`](nav_signals.py) | **The encoding.** Standalone, standard library only|
| [`haptics_nav.py`](haptics_nav.py) | GUI demo |
| [`haptic_ble.py`](haptic_ble.py) | BLE transport from python script to haptics |
| [`haptic_cli.py`](haptic_cli.py) | Interactive console for manually triggering any signal |
| [`run_study.py`](run_study.py) | Plays a fixed study block (c1s1 / c1s2 / c2s1 / c2s2) |

Haptic harness: 6 actuators in one row, labelled 1-6 left to right. 
## Before a study
Make sure the device can be connected to and that each motor vibrates by running `python haptic_ble.py`. If all actuators are buzzing, then you should be good to go. 

## Running a study
When running a study, you only need to use two of the 5 python scripts: `haptic_cli.py` and `run_study.py`.
#### User Study Familiarization
1. Start with `python haptic_cli.py` to manually trigger signals in either condition 1 (c1) or 2 (c2). **Example**: `c1 obstacle` plays condition 1's haptic signal.
2. Ask the user if they would like more practice and play any additional signals if necessary.
#### Run an actual study with user
1. Run a study by running the following command `python run_study.py c#s#`, where # indicates the condition and sequence numbers, respectively. **Example**: `python run_study.py c1s1` runs Trial C1S1. 
2. The user's verbal answers are recorded in the spreadsheet, ICRA 2026 Haptic Navigation - User Study > Seated Signal Sequences.

## Troubleshooting
If a haptic device fails to connect to Bluetooth on your laptop/computer, try the following:
1. Unplugging and replugging the battery.
2. Swapping out the battery. 
3. Swapping out the device with another one. Note: the MAC address will be different. 

If one of the motors on the haptic device isn't working, 
1. Swap the haptic device with another one. 
2. Let Aaron know to repair/replace. 
---

## The encoding

### Condition 1 — one signal per event, no warning

| Signal | Pattern | Length |
| --- | --- | --- |
| `obstacle` | all 6, 3 quick pulses — 80 ms on / 150 ms off | 0.54 s |
| `turn_left` | **motor 1 alone** — the left edge, no gradient | 0.30 s |
| `turn_right` | **motor 6 alone** — the right edge, no gradient | 0.30 s |
| `arrive` | all 6, one long buzz | 1.00 s |

### Condition 2 — warning, timer, then the signal

| Signal | Warning | Body | Length |
| --- | --- | --- | --- |
| `obstacle_left` | one 300 ms buzz on **motor 1** | `turn_right` gradient ×2 | 3.66 s |
| `obstacle_right` | one 300 ms buzz on **motor 6** | `turn_left` gradient ×2 | 3.66 s |
| `turn_left` | one buzz on **motor 1** | 3 more buzzes on motor 1 | 3.60 s |
| `turn_right` | one buzz on **motor 6** | 3 more buzzes on motor 6 | 3.60 s |
| `arrive` | all 6 buzz, 300 ms | one long buzz, same as C1's | 3.30 s |


## Running the GUI simulation demo

```bash
# BLE, condition 1
python .\haptics_nav.py --backend ble --c 1
# BLE, condition 2
python .\haptics_nav.py --backend ble --c 2
```
