# Two-Wheel Differential-Drive Maze Robot

This repository contains firmware and CAD files for a two-wheel differential-drive maze-solving robot built around the Adafruit Feather ESP32.

## Repository focus

The main working directory is `firmware/`, and the main purpose of this repository is firmware development.

- PlatformIO should be opened from the `firmware/` directory.
- CAD source files belong in `cad/source/`.
- Exported CAD/manufacturing files (for example STEP, STL, or PDF drawings) belong in `cad/exports/`.
- Maze-solving logic is planned but not yet implemented.

## Structure

```text
robot-project/
├── cad/
│   ├── source/
│   └── exports/
└── firmware/
    ├── platformio.ini
    ├── src/
    ├── include/
    ├── lib/
    └── test/
```
