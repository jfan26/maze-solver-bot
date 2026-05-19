#include <Arduino.h>

#include "config.h"
#include "motor_driver.h"
#include "sensors.h"
#include "robot_state.h"
#include "maze_solver.h"

static RobotPose g_pose{};
static SensorReadings g_sensorReadings{};

void setup() {
  Serial.begin(115200);

  initMotors();
  initSensors();

  // TODO: Initialize wheel encoders and differential-drive odometry state.
  // TODO: Initialize maze solver state machine and target cell policy.
}

void loop() {
  updateSensors();

  g_sensorReadings.frontDistanceM = getFrontDistance();
  g_sensorReadings.leftDistanceM = getLeftDistance();
  g_sensorReadings.rightDistanceM = getRightDistance();

  // TODO: Add closed-loop differential-drive control based on velocity targets.
  // TODO: Call maze solver update to select motion commands for cell traversal.

  updateMazeSolver(g_pose, g_sensorReadings);

  delay(CONTROL_LOOP_PERIOD_MS);
}
