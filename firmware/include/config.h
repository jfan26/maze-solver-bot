#pragma once

#include <Arduino.h>

// TODO: Replace with actual left motor driver input pins.
constexpr int LEFT_MOTOR_IN1_PIN = -1;
constexpr int LEFT_MOTOR_IN2_PIN = -1;

// TODO: Replace with actual right motor driver input pins.
constexpr int RIGHT_MOTOR_IN1_PIN = -1;
constexpr int RIGHT_MOTOR_IN2_PIN = -1;

// TODO: Set PWM-capable pins if the selected motor driver requires PWM control.
constexpr int LEFT_MOTOR_PWM_PIN = -1;
constexpr int RIGHT_MOTOR_PWM_PIN = -1;

// TODO: Set I2C pins for ToF sensors if custom pin mapping is required.
constexpr int TOF_SDA_PIN = -1;
constexpr int TOF_SCL_PIN = -1;

// TODO: Set XSHUT pins for each ToF sensor to allow individual addressing.
constexpr int TOF_FRONT_XSHUT_PIN = -1;
constexpr int TOF_LEFT_XSHUT_PIN = -1;
constexpr int TOF_RIGHT_XSHUT_PIN = -1;

// TODO: Set encoder pins if wheel encoders are present in the drivetrain.
constexpr int LEFT_ENCODER_A_PIN = -1;
constexpr int LEFT_ENCODER_B_PIN = -1;
constexpr int RIGHT_ENCODER_A_PIN = -1;
constexpr int RIGHT_ENCODER_B_PIN = -1;

// TODO: Update robot geometry constants after mechanical design is finalized.
constexpr float WHEEL_RADIUS_M = 0.030f;
constexpr float WHEELBASE_M = 0.120f;

// TODO: Tune control loop timing for motion/sensing requirements.
constexpr unsigned long CONTROL_LOOP_PERIOD_MS = 20;
