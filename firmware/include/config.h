#pragma once

#include <Arduino.h>

// Motor driver pins.
// Wiring reference:
//   Motor A: AIN1 -> GPIO 32, AIN2 -> GPIO 14
//   Motor B: BIN1 -> GPIO 33, BIN2 -> GPIO 15
// Current mapping assumption:
//   Motor A = LEFT motor, Motor B = RIGHT motor.
// If physical testing shows left/right swapped, only update these constants.
constexpr uint8_t LEFT_MOTOR_IN1 = 32;   // AIN1
constexpr uint8_t LEFT_MOTOR_IN2 = 14;   // AIN2
constexpr uint8_t RIGHT_MOTOR_IN1 = 33;  // BIN1
constexpr uint8_t RIGHT_MOTOR_IN2 = 15;  // BIN2 (ESP32 strapping pin)

// Optional inversion flags, to be adjusted after physical testing.
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;

// ToF XSHUT pins.
// WARNING: GPIO 12 is an ESP32 strapping pin and can affect boot if external
// hardware drives it to an unexpected level during reset.
constexpr uint8_t TOF_FRONT_XSHUT = 12;  // ESP32 strapping pin
constexpr uint8_t TOF_LEFT_XSHUT = 27;
constexpr uint8_t TOF_RIGHT_XSHUT = 13;

// ToF I2C addresses after reassignment.
constexpr uint8_t TOF_FRONT_ADDR = 0x30;
constexpr uint8_t TOF_LEFT_ADDR = 0x31;
constexpr uint8_t TOF_RIGHT_ADDR = 0x32;

// I2C pins.
// Use Feather ESP32 defaults unless these are changed later.
constexpr int I2C_SDA_PIN = SDA;
constexpr int I2C_SCL_PIN = SCL;

// VL53L0X continuous-read period. The control loop is 20 ms, so 30 ms gives
// fresher distance updates than the previous 50 ms without being too aggressive.
constexpr uint16_t TOF_CONTINUOUS_PERIOD_MS = 30;

// Motor speed limits.
constexpr int MOTOR_MIN_SPEED = -255;
constexpr int MOTOR_MAX_SPEED = 255;


// Manual calibration defaults for open-loop movement tests.
// Increase/decrease durations to tune distance and angle.
constexpr int CAL_MOVE_SPEED = 140;

// Straight-line forward calibration. Tune these independently if the robot
// consistently curves when both motors are commanded to the same PWM value.
constexpr int CAL_FORWARD_LEFT_SPEED = 140;
constexpr int CAL_FORWARD_RIGHT_SPEED = 140;

// Autonomous wall-following should usually be slower than manual straight-line
// tests, otherwise the ToF feedback cannot steer before the robot scrapes a wall.
constexpr int WALL_FOLLOW_FORWARD_BASE_LEFT_SPEED = 105;
constexpr int WALL_FOLLOW_FORWARD_BASE_RIGHT_SPEED = 105;
constexpr int WALL_FOLLOW_FORWARD_SPEED_SCALE_PERCENT = 100;
constexpr int WALL_FOLLOW_FORWARD_LEFT_SPEED =
    (WALL_FOLLOW_FORWARD_BASE_LEFT_SPEED * WALL_FOLLOW_FORWARD_SPEED_SCALE_PERCENT + 50) / 100;
constexpr int WALL_FOLLOW_FORWARD_RIGHT_SPEED =
    (WALL_FOLLOW_FORWARD_BASE_RIGHT_SPEED * WALL_FOLLOW_FORWARD_SPEED_SCALE_PERCENT + 50) / 100;
constexpr int WALL_FOLLOW_DRIVE_SPEED_SCALE_PERCENT = 81;
constexpr int WALL_FOLLOW_DRIVE_LEFT_SPEED =
    (WALL_FOLLOW_FORWARD_LEFT_SPEED * WALL_FOLLOW_DRIVE_SPEED_SCALE_PERCENT + 50) / 100;
constexpr int WALL_FOLLOW_DRIVE_RIGHT_SPEED =
    (WALL_FOLLOW_FORWARD_RIGHT_SPEED * WALL_FOLLOW_DRIVE_SPEED_SCALE_PERCENT + 50) / 100;

// This is still used by manual f/b tests, but the autonomous wall follower no
// longer drives forward for one fixed cell-sized duration.
constexpr uint32_t CAL_FORWARD_MS = 600;
constexpr int CAL_TURN_SPEED = 140;

// Separate left/right turn calibration
constexpr uint32_t CAL_TURN_LEFT_90_MS = 540;
constexpr uint32_t CAL_TURN_RIGHT_90_MS = 430;

// 360 command in manual mode spins clockwise, so base it on right-turn timing
constexpr uint32_t CAL_TURN_360_MS = 4 * CAL_TURN_RIGHT_90_MS;

// Wi-Fi command bridge (optional).
// Networking mode for wireless command/control over TCP.
enum class WifiMode : uint8_t {
  Station = 0,      // Join existing Wi-Fi network.
  AccessPoint = 1,  // Broadcast robot-hosted Wi-Fi network.
};
constexpr WifiMode WIFI_MODE = WifiMode::AccessPoint;

// Station mode credentials (used when WIFI_MODE == WifiMode::Station).
constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASS[] = "YOUR_WIFI_PASSWORD";

// Access-point mode settings (used when WIFI_MODE == WifiMode::AccessPoint).
constexpr char WIFI_AP_SSID[] = "MazeSolverBot";
constexpr char WIFI_AP_PASS[] = "maze1234";

constexpr uint16_t WIFI_CMD_PORT = 2323;

// Basic obstacle threshold for early testing.
constexpr uint16_t FRONT_OBSTACLE_THRESHOLD_MM = 150;

// Control loop period.
constexpr unsigned long CONTROL_LOOP_PERIOD_MS = 20;

// Wall-following behavior. Tune these after the manual motion calibration.
// Set to false for right-hand wall following.
constexpr bool WALL_FOLLOW_LEFT_HAND = true;

// Distances are in meters because the sensor helper functions return meters.
constexpr float WALL_FOLLOW_FRONT_BLOCKED_M = FRONT_OBSTACLE_THRESHOLD_MM / 1000.0f;

// Empirically tuned side-wall distances. These are measured from the sensor to
// the wall, not from the robot body edge. Shift them outward to account for the
// sensor sitting about 10 cm inboard from the body side.
constexpr float WALL_FOLLOW_TARGET_SIDE_M = 0.145f;

// Treat the followed side as an opening only when it stretches well beyond the
// normal tracking distance; this is the threshold that triggers left/right turn
// decisions at side branches.
constexpr float WALL_FOLLOW_SIDE_OPEN_M = 0.27f;

// When the wall follower decides to take a side opening, keep driving forward
// before the turn and again after the turn so the robot commits farther into the
// branch. Use a longer post-turn advance to carry the whole robot deeper into
// the new corridor. One "pulse" here is one control-loop update.
constexpr uint16_t WALL_FOLLOW_OPENING_PRE_TURN_ADVANCE_TICKS = 36;
constexpr uint32_t WALL_FOLLOW_OPENING_PRE_TURN_ADVANCE_MS =
    static_cast<uint32_t>(WALL_FOLLOW_OPENING_PRE_TURN_ADVANCE_TICKS) * CONTROL_LOOP_PERIOD_MS;
constexpr uint16_t WALL_FOLLOW_OPENING_POST_TURN_ADVANCE_TICKS =
    2 * WALL_FOLLOW_OPENING_PRE_TURN_ADVANCE_TICKS;
constexpr uint32_t WALL_FOLLOW_OPENING_POST_TURN_ADVANCE_MS =
    static_cast<uint32_t>(WALL_FOLLOW_OPENING_POST_TURN_ADVANCE_TICKS) * CONTROL_LOOP_PERIOD_MS;

// Cap how much the controller may steer toward the followed wall in a single
// update. Positive correction always means "arc toward the followed wall" for
// both left-hand and right-hand following. Set to 0 to disable those nudges.
constexpr int WALL_FOLLOW_MAX_TOWARD_WALL_CORRECTION = 18;

// Leave a small neutral band around the target distance so the robot does not
// keep flipping between "steer toward wall" and "steer away from wall" around
// the setpoint. Use a larger far-side threshold so it keeps going straight until
// the followed wall is clearly farther away.
constexpr float WALL_FOLLOW_TARGET_LEEWAY_M = 0.005f;
constexpr float WALL_FOLLOW_TOWARD_WALL_LEEWAY_M = 0.035f;

// Continuous proportional/derivative steering correction while driving forward
// along a side wall. These are intentionally stronger than before because the
// old gains produced only tiny PWM differences at realistic distance errors.
constexpr float WALL_FOLLOW_KP = 900.0f;
constexpr float WALL_FOLLOW_KD = 1600.0f;
constexpr int WALL_FOLLOW_CORRECTION_LIMIT = 25;
constexpr int WALL_FOLLOW_MIN_ACTIVE_FORWARD_SPEED = 85;
constexpr float WALL_FOLLOW_DISTANCE_DEADBAND_M = 0.003f;

// Emit steering telemetry while driving forward so wall-follow tuning can be
// correlated with the live ToF readings.
constexpr bool WALL_FOLLOW_LOG_CORRECTIONS = true;
constexpr uint32_t WALL_FOLLOW_LOG_PERIOD_MS = 100;

// Recovery for steady three-sensor readings. If front/left/right stay nearly
// unchanged for this long, treat it as stalled motion: back up, then turn away
// from the followed wall before re-evaluating.
constexpr float WALL_FOLLOW_STUCK_DELTA_M = 0.015f;
constexpr uint32_t WALL_FOLLOW_STUCK_MS = 500;
constexpr int WALL_FOLLOW_BACKUP_SPEED = 125;
constexpr uint32_t WALL_FOLLOW_BACKUP_MS = 120;
constexpr int WALL_FOLLOW_RECOVERY_TURN_SPEED = 120;
constexpr uint32_t WALL_FOLLOW_RECOVERY_TURN_MS = 50;

// Short pause after each turn or backup move to reduce overshoot before reading ToF again.
constexpr uint32_t WALL_FOLLOW_SETTLE_MS = 60;
constexpr uint32_t WALL_FOLLOW_TURN_AROUND_MS = 2 * CAL_TURN_RIGHT_90_MS;
