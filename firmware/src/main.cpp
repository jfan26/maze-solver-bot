#include <Arduino.h>

#include "config.h"
#include "maze_solver.h"
#include "motor_driver.h"
#include "robot_state.h"
#include "sensors.h"

// Choose bring-up mode.
constexpr bool RUN_HARDWARE_SMOKE_TEST = false;
constexpr bool RUN_MANUAL_CONTROL_TEST = true;

static RobotPose g_pose{};
static SensorReadings g_sensorReadings{};

namespace {

void runMotorPhase(const char* label, int left, int right, uint32_t durationMs) {
  Serial.printf("[MOTOR] %s | left=%d right=%d duration=%lu\n", label, left, right,
                static_cast<unsigned long>(durationMs));
  setMotorSpeeds(left, right);
  delay(durationMs);
  stopMotors();
  delay(200);
}

void runHardwareSmokeTestLoop() {
  static uint32_t lastSensorPrintMs = 0;
  static uint32_t lastSequenceStartMs = 0;
  static bool sequenceDone = false;

  updateSensors();

  if (millis() - lastSensorPrintMs >= 200) {
    lastSensorPrintMs = millis();
    Serial.printf("[TOF] front=%.3fm left=%.3fm right=%.3fm\n", getFrontDistance(),
                  getLeftDistance(), getRightDistance());
  }

  if (!sequenceDone && millis() - lastSequenceStartMs > 1500) {
    sequenceDone = true;

    constexpr int testSpeed = 120;
    constexpr uint32_t phaseMs = 700;

    runMotorPhase("left-forward", testSpeed, 0, phaseMs);
    runMotorPhase("left-reverse", -testSpeed, 0, phaseMs);
    runMotorPhase("right-forward", 0, testSpeed, phaseMs);
    runMotorPhase("right-reverse", 0, -testSpeed, phaseMs);
    runMotorPhase("both-forward", testSpeed, testSpeed, phaseMs);
    runMotorPhase("both-reverse", -testSpeed, -testSpeed, phaseMs);

    Serial.println("[SMOKE] motor test sequence complete; continuing ToF stream only.");
  }
}

void printManualHelp() {
  Serial.println("[MANUAL] Commands:");
  Serial.println("  h: help");
  Serial.println("  s: stop motors");
  Serial.println("  f: forward 0.50 m (timed)");
  Serial.println("  b: backward 0.50 m (timed)");
  Serial.println("  l: turn left 90 deg (timed)");
  Serial.println("  r: turn right 90 deg (timed)");
  Serial.println("  z: turn left 360 deg (timed)");
  Serial.println("  x: turn right 360 deg (timed)");
  Serial.println("  1: left motor forward pulse");
  Serial.println("  2: right motor forward pulse");
}

uint32_t forwardDistanceToMs(float distanceMeters) {
  if (CAL_FORWARD_SPEED_MPS <= 0.001f) {
    return 0;
  }
  return static_cast<uint32_t>((distanceMeters / CAL_FORWARD_SPEED_MPS) * 1000.0f);
}

void executeManualCommand(char c) {
  switch (c) {
    case 'h':
      printManualHelp();
      break;
    case 's':
      stopMotors();
      Serial.println("[MANUAL] stop");
      break;
    case 'f':
      runMotorPhase("forward-0.50m", CAL_LINEAR_TEST_SPEED, CAL_LINEAR_TEST_SPEED,
                    forwardDistanceToMs(0.50f));
      break;
    case 'b':
      runMotorPhase("backward-0.50m", -CAL_LINEAR_TEST_SPEED, -CAL_LINEAR_TEST_SPEED,
                    forwardDistanceToMs(0.50f));
      break;
    case 'l':
      runMotorPhase("turn-left-90", -CAL_TURN_TEST_SPEED, CAL_TURN_TEST_SPEED,
                    CAL_TURN_90_MS);
      break;
    case 'r':
      runMotorPhase("turn-right-90", CAL_TURN_TEST_SPEED, -CAL_TURN_TEST_SPEED,
                    CAL_TURN_90_MS);
      break;
    case 'z':
      runMotorPhase("turn-left-360", -CAL_TURN_TEST_SPEED, CAL_TURN_TEST_SPEED,
                    CAL_TURN_360_MS);
      break;
    case 'x':
      runMotorPhase("turn-right-360", CAL_TURN_TEST_SPEED, -CAL_TURN_TEST_SPEED,
                    CAL_TURN_360_MS);
      break;
    case '1':
      runMotorPhase("left-forward-pulse", CAL_LINEAR_TEST_SPEED, 0, 400);
      break;
    case '2':
      runMotorPhase("right-forward-pulse", 0, CAL_LINEAR_TEST_SPEED, 400);
      break;
    default:
      Serial.printf("[MANUAL] unknown command: %c\n", c);
      printManualHelp();
      break;
  }
}

void runManualControlLoop() {
  static uint32_t lastSensorPrintMs = 0;

  updateSensors();

  if (millis() - lastSensorPrintMs >= 250) {
    lastSensorPrintMs = millis();
    Serial.printf("[TOF] front=%.3fm left=%.3fm right=%.3fm\n", getFrontDistance(),
                  getLeftDistance(), getRightDistance());
  }

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      continue;
    }
    executeManualCommand(c);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("[BOOT] init motors/sensors");
  initMotors();
  initSensors();

  if (RUN_HARDWARE_SMOKE_TEST) {
    Serial.println("[SMOKE] Hardware smoke test mode enabled");
  }
  if (RUN_MANUAL_CONTROL_TEST) {
    Serial.println("[MANUAL] Manual control mode enabled");
    printManualHelp();
  }

  // TODO: Initialize wheel encoders and differential-drive odometry state.
  // TODO: Initialize maze solver state machine and target cell policy.
}

void loop() {
  if (RUN_HARDWARE_SMOKE_TEST) {
    runHardwareSmokeTestLoop();
    delay(10);
    return;
  }

  if (RUN_MANUAL_CONTROL_TEST) {
    runManualControlLoop();
    delay(10);
    return;
  }

  updateSensors();

  g_sensorReadings.frontDistanceM = getFrontDistance();
  g_sensorReadings.leftDistanceM = getLeftDistance();
  g_sensorReadings.rightDistanceM = getRightDistance();

  updateMazeSolver(g_pose, g_sensorReadings);

  delay(CONTROL_LOOP_PERIOD_MS);
}
