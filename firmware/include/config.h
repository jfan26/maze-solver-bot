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

// Motor speed limits.
constexpr int MOTOR_MIN_SPEED = -255;
constexpr int MOTOR_MAX_SPEED = 255;


// Manual calibration defaults for open-loop movement tests.
// Increase/decrease durations to tune distance and angle.
constexpr int CAL_MOVE_SPEED = 140;
constexpr uint32_t CAL_FORWARD_MS = 600;
constexpr int CAL_TURN_SPEED = 140;
constexpr uint32_t CAL_TURN_90_MS = 340;
constexpr uint32_t CAL_TURN_360_MS = 4 * CAL_TURN_90_MS;

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
