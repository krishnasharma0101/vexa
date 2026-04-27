# Hardware Setup Guide

## Wiring Diagram

```
                    ┌──────────────────────┐
                    │    Raspberry Pi 5     │
                    │                       │
    ┌───────────────┤ GPIO 17 (pin 11)  ───┤──── Base (MG995)      signal
    │               ├ GPIO 27 (pin 13)  ───┤──── Shoulder (MG995)  signal
    │               ├ GPIO 22 (pin 15)  ───┤──── Elbow (MG995)     signal
    │               ├ GPIO 23 (pin 16)  ───┤──── Wrist Pitch (MG995) signal
    │               ├ GPIO 24 (pin 18)  ───┤──── Wrist Roll (MG90S)  signal
    │               ├ GPIO 25 (pin 22)  ───┤──── Gripper (MG996R)    signal
    │               │                       │
    │       ┌───────┤ GND (pin 6, 9, etc.) │
    │       │       │                       │
    │       │       │   CSI camera port     │◄─── RPi Camera Module (ribbon cable)
    │       │       └──────────────────────┘
    │       │
    │       │       ┌──────────────────────┐
    │       └──────►│        SMPS          │
    │    common GND │   5V – 6V / ≥ 5A    │
    │               │                       │
    │               │  V+  ────────────────┤──── All servo RED wires
    │               │  GND ────────────────┤──── All servo BROWN/BLACK wires
    │               └──────────────────────┘
    │
    └── CRITICAL: RPi GND and SMPS GND must be connected together!
```

## Per-Servo Wiring

Each servo has 3 wires:

| Wire Colour | Connects To |
|---|---|
| **Red** (V+) | SMPS positive (5–6 V) |
| **Brown / Black** (GND) | SMPS ground |
| **Orange / Yellow** (Signal) | RPi GPIO pin (see table below) |

## GPIO Pin Assignment

| Joint | Servo | BCM GPIO | Physical Pin |
|---|---|---|---|
| Base | MG995 | 17 | 11 |
| Shoulder | MG995 | 27 | 13 |
| Elbow | MG995 | 22 | 15 |
| Wrist Pitch | MG995 | 23 | 16 |
| Wrist Roll | MG90S | 24 | 18 |
| Gripper | MG996R | 25 | 22 |

## Power Supply Requirements

- **Voltage**: 5 V to 6 V DC (servos are rated up to 6 V)
- **Current**: At least **5 A** recommended
  - MG995: up to 1.2 A stall × 4 = 4.8 A worst case
  - MG996R: up to 1.5 A stall
  - MG90S: up to 0.7 A stall
  - Total worst case: ~7 A (all stalling simultaneously, rare)
  - Normal operation: 2–3 A
- Use a good quality SMPS (Switched-Mode Power Supply)

## Camera Setup

1. Power off the RPi 5
2. Gently lift the CSI connector latch on the RPi 5
3. Insert the camera ribbon cable (contacts facing the PCB)
4. Press the latch back down
5. Power on and test: `libcamera-still -o test.jpg`

## First-Time Software Setup (on RPi 5)

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y python3-pip python3-venv libcap-dev

# Create virtual environment
cd ~/Desktop/robotic\ arm
python3 -m venv venv --system-site-packages
source venv/bin/activate

# Install Python packages
pip install -r requirements.txt

# Optional: enable pigpio for better PWM
sudo apt install -y pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

## Testing Sequence

1. **Verify GPIO**: `pinout` (shows RPi pinout diagram)
2. **Test servos**: `python main.py --test-servos`
3. **Test camera**: `python main.py --test-camera`
4. **Calibrate servos**: `python calibrate.py --servos`
5. **Calibrate camera**: `python calibrate.py --camera`
6. **Run single cycle**: `python main.py --single`
7. **Run continuous**: `python main.py`

## Troubleshooting

| Problem | Solution |
|---|---|
| Servos jitter | Install & start `pigpiod` for DMA-timed PWM |
| Servo doesn't move | Check signal wire on correct GPIO pin |
| All servos dead | Check SMPS power and common GND connection |
| Camera not found | Re-seat ribbon cable, run `libcamera-still -o test.jpg` |
| Import errors | Activate venv: `source venv/bin/activate` |
| Permission denied (GPIO) | Run with `sudo` or add user to `gpio` group |
