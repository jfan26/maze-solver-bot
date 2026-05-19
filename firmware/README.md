# Firmware (PlatformIO)

This directory is the PlatformIO firmware project for the maze-solving robot.

## Project root for PlatformIO

The PlatformIO project root is `firmware/`.

## Commands

Build:

```bash
cd firmware
pio run
```

Upload:

```bash
pio run -t upload
```

Serial monitor:

```bash
pio device monitor -b 115200
```

## Manual control test mode (interactive)

The current `src/main.cpp` enables `RUN_MANUAL_CONTROL_TEST = true`.

### Bring-up steps

1. Connect board and identify port:

```bash
cd firmware
pio device list
```

2. Upload firmware:

```bash
pio run -t upload
```

3. Open serial monitor:

```bash
pio device monitor -b 115200
```

### Manual commands

Type one character and press Enter:

- `h` help
- `s` stop motors
- `f` forward 0.50 m (timed)
- `b` backward 0.50 m (timed)
- `l` left turn 90° (timed)
- `r` right turn 90° (timed)
- `z` left turn 360° (timed)
- `x` right turn 360° (timed)
- `1` left motor pulse
- `2` right motor pulse

The monitor also prints periodic ToF readings.

## What to calibrate

Edit `include/config.h`:

- Direction and side mapping:
  - `LEFT_MOTOR_INVERTED`
  - `RIGHT_MOTOR_INVERTED`
  - `LEFT_MOTOR_IN1/IN2`, `RIGHT_MOTOR_IN1/IN2` if sides are swapped.

- Turn calibration:
  - `CAL_TURN_TEST_SPEED`
  - `CAL_TURN_90_MS`
  - `CAL_TURN_360_MS`

- Distance calibration:
  - `CAL_LINEAR_TEST_SPEED`
  - `CAL_FORWARD_SPEED_MPS`

Recommended process:

1. Use `1` and `2` to verify wheel-side mapping.
2. Use `l`/`r` and tune `CAL_TURN_90_MS` until turns are near 90°.
3. Use `z`/`x` and tune `CAL_TURN_360_MS` until turns are near 360°.
4. Run `f` for 0.50 m and adjust `CAL_FORWARD_SPEED_MPS` to match measured travel.

> Safety: keep the robot lifted during first motor-direction checks.

## Optional smoke-test mode

If needed, disable manual mode and enable smoke mode in `src/main.cpp`.
