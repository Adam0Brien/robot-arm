# Robot Arm (5-DOF)

Control a 5-DOF robotic arm from a Raspberry Pi using a PCA9685 servo driver. This repo provides a chainable Python API (`arm.py`) on top of the low-level servo controller (`setup.py`).

## Development workflow

Develop on your machine, then upload the project to the Pi.

**Upload code to the Pi** (run from your **local** machine):

```bash
sshpass -p 'YOUR_PASSWORD' scp -r /home/aobrien/Desktop/robot-arm student0@raspberrypi:~/
```

Then SSH into the Pi and run from `~/robot-arm`:

```bash
ssh student0@raspberrypi
cd ~/robot-arm
python3 main.py
```

## Project layout

| File        | Role |
|------------|------|
| `arm.py`   | Chainable API: `move_base()`, `move_shoulder()`, `move_elbow()`, `move_wrist()`, `move_gripper()`, `open_gripper()`, `close_gripper()`, `wait()`, `home()`, etc. |
| `setup.py` | Low-level servo controller (ServoCtrl, PCA9685). Used by `arm.py`. |
| `main.py`  | Entrypoint and example sequences. |
| `plan.json`| Optional saved pose sequence for the low-level plan replay feature. |

## Requirements

- Raspberry Pi with I2C enabled
- PCA9685 at address `0x40`, servos on 0–4 for the arm
- Python 3 with: `adafruit-circuitpython-servokit` (or `adafruit-motor`, `adafruit-pca9685`), `busio`, `board` (e.g. via Blinka on Pi)

## Example

```python
from arm import Arm

arm = Arm()
arm.home().wait(1).move_base(45).move_shoulder(60).wait(0.5).close_gripper().wait(1).open_gripper().home()
```
