# Buzz_Haptics

Haptic navigation encoding for a guide-dog robot. 

| File | Role |
| --- | --- |
| [`haptics_nav.py`](haptics_nav.py) | The encoding itself, plus a Tk visualizer and the scripted study demo |
| [`haptic_ble.py`](haptic_ble.py) | Synchronous BLE transport from python script to haptics |
| [`haptic_cli.py`](haptic_cli.py) | Interactive console for manually triggering any signal |

### Running the demo

```bash
# BLE, condition 1
python .\haptics_nav.py --backend ble --c 1                    
# BLE, condition 2 
python .\haptics_nav.py --backend ble --c 2
# GUI only, no haptics hardware                         
python haptics_nav.py --backend dry          
```

Note: to test if every motor is working via BLE, run `python haptic_ble.py`.

Note: to run the CLI console, just run `python haptic_cli.py`. 