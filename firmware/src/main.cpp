#include <Arduino.h>

#include "config.h"
#include "maze_solver.h"
#include "motor_driver.h"
#include "robot_state.h"
#include "sensors.h"

// Set to false when switching back to maze-solver behavior.
constexpr bool RUN_HARDWARE_SMOKE_TEST = true;

static RobotPose g_pose{};
static SensorReadings g_sensorReadings{};

namespace {

void runMotorPhase(const char* label, int left, int right, uint32_t durationMs) {
  Serial.printf("[MOTOR] %s | left=%d right=%d\n", label, left, right);
  setMotorSpeeds(left, right);
  delay(durationMs);
  stopMotors();
  delay(250);
}

void runHardwareSmokeTestLoop() {
  static uint32_t lastSensorPrintMs = 0;
  static uint32_t lastSequenceStartMs = 0;
  static bool sequenceDone = false;

  updateSensors();

  if (millis() - lastSensorPrintMs >= 200) {
    lastSensorPrintMs = millis();
    Serial.printf(
        "[TOF] front=%.3fm left=%.3fm right=%.3fm\n",
        getFrontDistance(),
        getLeftDistance(),
        getRightDistance());
  }

  if (!sequenceDone && millis() - lastSequenceStartMs > 1500) {
    sequenceDone = true;

    // Keep the robot lifted or wheels off the ground for this bring-up test.
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

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("[BOOT] init motors/sensors");
  initMotors();
  initSensors();

  if (RUN_HARDWARE_SMOKE_TEST) {
    Serial.println("[SMOKE] Hardware smoke test mode enabled");
    Serial.println("[SMOKE] Open serial monitor @115200 to view ToF and motor phases");
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

  updateSensors();

  g_sensorReadings.frontDistanceM = getFrontDistance();
  g_sensorReadings.leftDistanceM = getLeftDistance();
  g_sensorReadings.rightDistanceM = getRightDistance();

  // TODO: Add closed-loop differential-drive control based on velocity targets.
  // TODO: Call maze solver update to select motion commands for cell traversal.

  updateMazeSolver(g_pose, g_sensorReadings);

  delay(CONTROL_LOOP_PERIOD_MS);
}
