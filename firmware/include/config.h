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
constexpr int WALL_FOLLOW_FORWARD_LEFT_SPEED = 105;
constexpr int WALL_FOLLOW_FORWARD_RIGHT_SPEED = 105;

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

// Side-wall geometry calibration.
// The robot body is approximately an octagon with 50 mm inner radius/apothem.
// Side ToF readings are measured from the sensor lens, not from the outside edge
// of the robot. If the side sensor lens is recessed inward from the nearest body
// edge, include that inset here. With the default 45 mm inset and 30 mm desired
// body clearance, the side ToF target is 75 mm.
constexpr float ROBOT_INNER_RADIUS_M = 0.050f;
constexpr float SIDE_TOF_SENSOR_INSET_FROM_BODY_EDGE_M = 0.045f;
constexpr float WALL_FOLLOW_BODY_CLEARANCE_M = 0.030f;
constexpr float WALL_FOLLOW_TARGET_SIDE_M =
    WALL_FOLLOW_BODY_CLEARANCE_M + SIDE_TOF_SENSOR_INSET_FROM_BODY_EDGE_M;

// Treat the followed side as an opening only when the side reading is much larger
// than the normal wall-following distance. Lower this if the robot overshoots
// side openings; raise it if it falsely turns left/right while still in a corridor.
constexpr float WALL_FOLLOW_SIDE_OPEN_M = WALL_FOLLOW_TARGET_SIDE_M + 0.105f;

// Reject impossible/ambiguous side readings when deciding whether a side is
// open. This is important because the VL53L0X can report 0 mm or a huge
// out-of-range value when it is too close to a wall or times out. Those readings
// should NOT make a left-hand follower immediately command a left turn.
constexpr float WALL_FOLLOW_SIDE_MIN_VALID_M = 0.015f;
constexpr float WALL_FOLLOW_SIDE_MAX_OPEN_VALID_M = 1.000f;

// If true, left-hand following is prevented from steering farther left during
// forward correction. Leave this false for normal wall following; the side-open
// filter above is the actual fix for the false left-turn problem.
constexpr bool WALL_FOLLOW_DISABLE_LEFT_ARC_CORRECTION = false;

// Continuous proportional/derivative steering correction while driving forward
// along a side wall. These are intentionally stronger than before because the
// old gains produced only tiny PWM differences at realistic distance errors.
constexpr float WALL_FOLLOW_KP = 900.0f;
constexpr float WALL_FOLLOW_KD = 1600.0f;
constexpr int WALL_FOLLOW_CORRECTION_LIMIT = 65;
constexpr float WALL_FOLLOW_DISTANCE_DEADBAND_M = 0.003f;

// Emit steering telemetry while driving forward so wall-follow tuning can be
// correlated with the live ToF readings.
constexpr bool WALL_FOLLOW_LOG_CORRECTIONS = true;
constexpr uint32_t WALL_FOLLOW_LOG_PERIOD_MS = 100;

// Stuck/collision recovery. If all three ToF readings stay almost unchanged
// while the robot is trying to drive forward, assume the robot is pinned against
// something: back up briefly, then turn slightly away from the followed wall.
constexpr float WALL_FOLLOW_STUCK_DELTA_M = 0.006f;
constexpr float WALL_FOLLOW_STUCK_MAX_SENSOR_M = 1.00f;
constexpr uint32_t WALL_FOLLOW_STUCK_MS = 180;
constexpr int WALL_FOLLOW_BACKUP_SPEED = 125;
constexpr uint32_t WALL_FOLLOW_BACKUP_MS = 120;
constexpr int WALL_FOLLOW_RECOVERY_TURN_SPEED = 120;
constexpr uint32_t WALL_FOLLOW_RECOVERY_TURN_MS = 75;

// Short pause after each turn/recovery move to reduce overshoot before reading ToF again.
constexpr uint32_t WALL_FOLLOW_SETTLE_MS = 60;
constexpr uint32_t WALL_FOLLOW_TURN_AROUND_MS = 2 * CAL_TURN_RIGHT_90_MS;
