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
pio device monitor
```

## Current status

The current firmware is only a lightweight skeleton.

Planned modules:

- Motor control
- Sensor handling
- Robot state estimation
- Maze solving
