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

// Basic obstacle threshold for early testing.
constexpr uint16_t FRONT_OBSTACLE_THRESHOLD_MM = 150;

// Control loop period.
constexpr unsigned long CONTROL_LOOP_PERIOD_MS = 20;

// --- Manual calibration entries (edit these during bench calibration) ---
constexpr int CAL_LINEAR_TEST_SPEED = 120;
constexpr int CAL_TURN_TEST_SPEED = 120;

// Durations used by simple open-loop test motions.
constexpr uint32_t CAL_TURN_90_MS = 500;
constexpr uint32_t CAL_TURN_360_MS = 2000;

// Distance model for timed forward moves.
// Approximate forward speed in m/s at CAL_LINEAR_TEST_SPEED.
constexpr float CAL_FORWARD_SPEED_MPS = 0.20f;
