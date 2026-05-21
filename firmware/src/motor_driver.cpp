#include "motor_driver.h"

#include <Arduino.h>

#include "config.h"

namespace {

int clampMotorCommand(int command) {
  if (command > MOTOR_MAX_SPEED) {
    return MOTOR_MAX_SPEED;
  }
  if (command < MOTOR_MIN_SPEED) {
    return MOTOR_MIN_SPEED;
  }
  return command;
}

int applyLeftStraightTrim(int left, int right) {
  const bool sameDirection = (left > 0 && right > 0) || (left < 0 && right < 0);
  if (!sameDirection) {
    return left;
  }

  if (left > 0) {
    return clampMotorCommand(left + CAL_LEFT_TRIM_FORWARD);
  }

  return clampMotorCommand(left - CAL_LEFT_TRIM_REVERSE);
}

void writeSingleMotor(int in1Pin, int in2Pin, int command, bool inverted) {
  int output = clampMotorCommand(command);
  if (inverted) {
    output = -output;
  }

  if (output > 0) {
    analogWrite(in1Pin, output);
    analogWrite(in2Pin, 0);
  } else if (output < 0) {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, -output);
  } else {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, 0);
  }
}

}  // namespace

void initMotors() {
  // WARNING: GPIO 15 is an ESP32 strapping pin and can affect boot if external
  // hardware drives it to an unexpected level during reset.
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  // As early as possible in setup(), ensure safe stopped state.
  stopMotors();
}

void setMotorSpeeds(int left, int right) {
  const int adjustedLeft = applyLeftStraightTrim(left, right);
  writeSingleMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, adjustedLeft, LEFT_MOTOR_INVERTED);
  writeSingleMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, right, RIGHT_MOTOR_INVERTED);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_IN1, 0);
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(RIGHT_MOTOR_IN1, 0);
  analogWrite(RIGHT_MOTOR_IN2, 0);
}
