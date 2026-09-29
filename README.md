# ESP32 Ultrasonic Radar & Pan-Tilt Laser Turret

Two small embedded projects built around an **ESP32** and standard RC hobby servos, each paired with a **Python desktop app** over USB serial:

1. **Ultrasonic radar**: an HC-SR04 sensor on a pan servo sweeps 15°–165°. The ESP32 streams `(angle, distance)` pairs to the PC, and a pygame app draws a live radar screen (semicircle PPI) with fading echoes.
2. **Laser turret (pan-tilt)**: the ESP32 takes pan/tilt/laser commands over serial and drives two servos to the target angle along a smooth **trapezoidal velocity profile**, with a safety timeout that switches the laser off. It is meant to be driven by an external vision/tracking app.

> Status: firmware compiles cleanly (`esp32:esp32:esp32`, arduino-esp32 core 3.3.x). The PC apps are tested end-to-end with the built-in `--demo` simulator. Bench testing on real hardware is in progress.

## Repository layout

| Path | What it is |
|---|---|
| `UltrasonicRadar/` | Sweeping radar firmware for SG90/MG90S-class servos (2° step, 60 ms/step, ≈4.6 s per sweep) |
| `UltrasonicRadarMG995/` | Same protocol, tuned for the MG995: configurable pulse range (`writeMicroseconds`), servo settle delay, median of 3 pings per angle, 3° step, 300 cm range |
| `UltrasonicDistance/` | Fixed sensor with no servo, for testing the HC-SR04 alone |
| `ServoCalibration/` | Helper sketch that finds a servo's real pulse range and detects fake "360° continuous" MG995 clones |
| `LaserTurretControl/` | Pan-tilt + laser firmware: CSV command parser, motion profiling, safety timeout |
| `RadarDisplay/radar_display.py` | Live radar screen (pygame + pyserial) for the sweeping firmware |
| `RadarDisplay/distance_display.py` | Single-beam display for `UltrasonicDistance` |
| `HUONG_DAN.txt` | Step-by-step user guide in Vietnamese (wiring, flashing, running) |

## Hardware

- ESP32 DevKit (WROOM-32)
- HC-SR04 ultrasonic sensor
- RC servos: SG90 / MG90S / MG995 / MG996R
- A low-power laser module (< 5 mW, Class 1/2), only for the turret
- A separate 5 V (≥ 2 A) supply for the servos, with ground shared with the ESP32

| Signal | GPIO |
|---|---|
| Pan servo | 18 |
| Tilt servo (turret) | 19 |
| Laser enable (turret) | 21 |
| HC-SR04 TRIG | 25 |
| HC-SR04 ECHO | 26, **through a 1 kΩ / 2 kΩ voltage divider** (ECHO outputs 5 V, but ESP32 pins are 3.3 V only) |

Put a 470–1000 µF electrolytic capacitor across the servo supply and a 100 nF ceramic capacitor at the HC-SR04 VCC pin. Servo inrush current otherwise shows up as noisy distance readings.

## Serial protocols (115200 baud, newline-terminated)

**Radar (ESP32 → PC)**
```
<angle_deg>,<distance_cm>      e.g. 90,123.4      (-1 = no echo / out of 2–400 cm range)
```
**Distance only (ESP32 → PC)**
```
<distance_cm>
```
**Turret (PC → ESP32)**
```
<pan_deg>,<tilt_deg>,<laser 0|1>   e.g. 90,45,1
STOP                               detach servos, laser off
```
The turret switches the laser off automatically if no command arrives for 2 s, so a crashed or disconnected PC can't leave the laser on.

## Design notes

- **Smooth motion**: the turret doesn't jump straight to each new target. It follows a trapezoidal profile (`MAX_SPEED_DEG_PER_S`, `MAX_ACCEL_DEG_PER_S2`), so a noisy target angle from a vision tracker doesn't make the servos jerk.
- **Robust ranging**: the MG995 build waits for the servo to settle before each ping and takes the median of 3 readings. This filters out spikes caused by vibration and electrical noise.
- **Display**: returns fade out over a configurable persistence time, the range can be zoomed live, and a `--mirror` flag handles servos mounted in reverse. Bluetooth COM ports are skipped automatically when auto-detecting the port.

## Getting started

### Firmware
1. Arduino IDE → Boards Manager → install **esp32** (Espressif), then install the **ESP32Servo** library.
2. Open the sketch folder, choose *ESP32 Dev Module* and the right COM port, then Upload.

Or with `arduino-cli`:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 UltrasonicRadar
arduino-cli upload  --fqbn esp32:esp32:esp32 -p COM5 UltrasonicRadar
```

### PC display
```bash
pip install pygame pyserial
python RadarDisplay/radar_display.py --demo          # simulated data, no hardware needed
python RadarDisplay/radar_display.py --port COM5     # real ESP32
```
Options: `--range <cm>` (initial display range, default 200), `--persist <s>` (echo fade time, default 6), `--mirror`.
Keys: `+`/`-` or mouse wheel to zoom, `C` to clear, `Esc` to quit.

## Safety

Use only low-power (< 5 mW) laser modules and never aim them at people, vehicles or aircraft. Set the `PAN/TILT_MIN/MAX` limits to match the real mechanical frame so the servos never stall against an end stop.

## Tech stack

C++ (Arduino / ESP32), ESP32Servo, Python 3.12, pygame, pyserial
