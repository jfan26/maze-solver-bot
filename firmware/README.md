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

## Hardware smoke test (motors + ToF)

The current `src/main.cpp` has `RUN_HARDWARE_SMOKE_TEST = true` for bring-up.

### 1) Connect board and identify serial port

```bash
cd firmware
pio device list
```

### 2) Build and upload

```bash
pio run -t upload
```

### 3) Open serial monitor

```bash
pio device monitor -b 115200
```

Expected startup logs include:

- `[BOOT] init motors/sensors`
- `[ToF] front initialized at 0x30`
- `[ToF] left initialized at 0x31`
- `[ToF] right initialized at 0x32`
- periodic `[TOF] front=... left=... right=...`

### 4) Motor phase test sequence (automatic, one-time)

After ~1.5s, motor phases are executed in this order:

1. left-forward
2. left-reverse
3. right-forward
4. right-reverse
5. both-forward
6. both-reverse

Each phase is printed as `[MOTOR] ...` and then stopped before the next one.

> Safety: lift the robot so wheels are off the ground during this test.

### 5) How to interpret results

- If a commanded side spins the opposite direction, toggle that side's inversion flag in `include/config.h`.
- If "left" commands move the physical right wheel (or vice versa), swap the left/right motor pin assignments in `include/config.h`.
- If any ToF sensor does not initialize, verify its XSHUT wire and the configured pin/address constants.

### 6) Exit smoke-test mode

Set `RUN_HARDWARE_SMOKE_TEST` to `false` in `src/main.cpp` to return to normal loop behavior.

## Current status

Early integration stage with hardware bring-up utilities enabled.
