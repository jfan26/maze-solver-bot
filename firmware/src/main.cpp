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

void printManualControlHelp() {
  Serial.println("[CTRL] Commands:");
  Serial.println("[CTRL]   h            -> help");
  Serial.println("[CTRL]   f            -> forward calibrated distance");
  Serial.println("[CTRL]   b            -> reverse calibrated distance");
  Serial.println("[CTRL]   l            -> turn left 90 deg");
  Serial.println("[CTRL]   r            -> turn right 90 deg");
  Serial.println("[CTRL]   t            -> turn 360 deg (CW)");
  Serial.println("[CTRL]   p            -> toggle ToF printout");
  Serial.println("[CTRL]   s            -> stop motors");
}

void executeManualCommand(char command, bool& tofPrintEnabled) {
  switch (command) {
    case 'h':
      printManualControlHelp();
      break;
    case 'f':
      runMotorPhase("forward calibrated", CAL_MOVE_SPEED, CAL_MOVE_SPEED, CAL_FORWARD_MS);
      break;
    case 'b':
      runMotorPhase("reverse calibrated", -CAL_MOVE_SPEED, -CAL_MOVE_SPEED, CAL_FORWARD_MS);
      break;
    case 'l':
      runMotorPhase("turn left 90", -CAL_TURN_SPEED, CAL_TURN_SPEED, CAL_TURN_90_MS);
      break;
    case 'r':
      runMotorPhase("turn right 90", CAL_TURN_SPEED, -CAL_TURN_SPEED, CAL_TURN_90_MS);
      break;
    case 't':
      runMotorPhase("turn 360", CAL_TURN_SPEED, -CAL_TURN_SPEED, CAL_TURN_360_MS);
      break;
    case 'p':
      tofPrintEnabled = !tofPrintEnabled;
      Serial.printf("[CTRL] ToF printout %s\n", tofPrintEnabled ? "enabled" : "disabled");
      break;
    case 's':
      stopMotors();
      Serial.println("[CTRL] stop");
      break;
    default:
      Serial.printf("[CTRL] Unknown command '%c'\n", command);
      printManualControlHelp();
      break;
  }
}

void processSerialCommands(bool& tofPrintEnabled) {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r' || c == ' ') {
      continue;
    }
    executeManualCommand(c, tofPrintEnabled);
  }
}

void runHardwareSmokeTestLoop() {
  static uint32_t lastSensorPrintMs = 0;
  static uint32_t lastSequenceStartMs = 0;
  static bool sequenceDone = false;
  static bool tofPrintEnabled = true;

  updateSensors();
  processSerialCommands(tofPrintEnabled);

  if (tofPrintEnabled && millis() - lastSensorPrintMs >= 200) {
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

    Serial.println("[SMOKE] motor test sequence complete; continuing ToF stream + manual controls.");
    printManualControlHelp();
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
