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

> USB serial monitor remains supported even when Wi-Fi command bridge is enabled.

## Hardware smoke test (motors + ToF)

The current `src/main.cpp` has `RUN_HARDWARE_SMOKE_TEST_AT_BOOT = false` by default.

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

Set `RUN_HARDWARE_SMOKE_TEST_AT_BOOT` to `true` in `src/main.cpp` to re-enable the automatic smoke-test sequence.


## Manual motion control in smoke-test mode

After the automatic phase sequence finishes, you can send single-character commands from the serial monitor to run calibrated motions:

- `l`: turn left 90°
- `r`: turn right 90°
- `t`: turn 360° clockwise
- `f`: move forward calibrated distance
- `b`: move backward calibrated distance
- `p`: toggle ToF printout
- `s`: stop motors
- `h`: print command help

## Optional wireless command control (Wi-Fi TCP bridge)

Smoke-test mode also supports wireless single-character commands over TCP:

1. Select a mode in `include/config.h`:
   - `WifiMode::AccessPoint` (default): robot broadcasts its own Wi-Fi SSID.
   - `WifiMode::Station`: robot joins your existing Wi-Fi using `WIFI_SSID`/`WIFI_PASS`.
2. Upload firmware (first-time upload is still easiest over USB).
3. Find the robot IP from startup logs:
   - AP mode: `[NET] AP IP=...` (usually `192.168.4.1`)
   - Station mode: `[NET] Wi-Fi connected, IP=...`
4. Connect from your computer:

```bash
nc <robot-ip> 2323
```

Then send the same commands (`h`, `f`, `b`, `l`, `r`, `t`, `p`, `s`) as serial.

### School-network note

If your school network blocks device onboarding, use `WifiMode::AccessPoint` so your laptop connects directly to the robot's broadcasted SSID (`WIFI_AP_SSID`) instead of campus Wi-Fi.

### Calibration entries to modify

Tune these constants in `include/config.h`:

- `CAL_MOVE_SPEED`: PWM command for straight moves
- `CAL_FORWARD_MS`: duration used for `f` and `b`
- `CAL_TURN_SPEED`: PWM command for turns
- `CAL_TURN_90_MS`: duration used for 90° turns (`l`/`r`)
- `CAL_TURN_360_MS`: duration used for 360° turn (`t`), default is `4 * CAL_TURN_90_MS`

Recommended workflow:

1. Start with low speed and short duration.
2. Run `l`/`r`, measure actual angle, and adjust `CAL_TURN_90_MS`.
3. Run `f`, measure distance, and adjust `CAL_FORWARD_MS`.
4. Re-upload and repeat until repeatable.

## Current status

Early integration stage with hardware bring-up utilities enabled.
