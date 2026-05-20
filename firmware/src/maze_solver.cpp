#include "maze_solver.h"

#include <Arduino.h>

#include "config.h"
#include "motor_driver.h"

namespace {

// This implementation uses a simple left-hand or right-hand wall-following rule.
// It assumes CAL_FORWARD_MS moves approximately one maze cell and CAL_TURN_90_MS
// turns approximately 90 degrees. Tune those first using smoke-test/manual mode.

enum class WallFollowPhase {
  Decide,
  DriveForward,
  TurnLeft,
  TurnRight,
  TurnAround,
  Settle,
  Stopped
};

MazeSolverState g_state{MazeStrategy::None, false};
WallFollowPhase g_phase = WallFollowPhase::Stopped;
WallFollowPhase g_nextPhaseAfterSettle = WallFollowPhase::Decide;
uint32_t g_phaseStartMs = 0;
uint32_t g_phaseDurationMs = 0;
uint32_t g_lastLogMs = 0;

bool isFiniteUsableDistance(float distanceM) {
  // VL53L0X returns very large distances when it is out of range. Treat those
  // as open space, not invalid. Only reject impossible/zero/NaN readings.
  return distanceM > 0.005f && distanceM == distanceM;
}

bool frontIsBlocked(const SensorReadings& sensors) {
  return isFiniteUsableDistance(sensors.frontDistanceM) &&
         sensors.frontDistanceM < WALL_FOLLOW_FRONT_BLOCKED_M;
}

bool frontIsOpen(const SensorReadings& sensors) {
  return !frontIsBlocked(sensors);
}

bool leftIsOpen(const SensorReadings& sensors) {
  return !isFiniteUsableDistance(sensors.leftDistanceM) ||
         sensors.leftDistanceM > WALL_FOLLOW_SIDE_OPEN_M;
}

bool rightIsOpen(const SensorReadings& sensors) {
  return !isFiniteUsableDistance(sensors.rightDistanceM) ||
         sensors.rightDistanceM > WALL_FOLLOW_SIDE_OPEN_M;
}

int clampCorrection(float correction) {
  if (correction > WALL_FOLLOW_CORRECTION_LIMIT) {
    return WALL_FOLLOW_CORRECTION_LIMIT;
  }
  if (correction < -WALL_FOLLOW_CORRECTION_LIMIT) {
    return -WALL_FOLLOW_CORRECTION_LIMIT;
  }
  return static_cast<int>(correction);
}

void logState(const char* message, const SensorReadings& sensors) {
  const uint32_t now = millis();
  if (now - g_lastLogMs < 100) {
    return;
  }
  g_lastLogMs = now;
  Serial.printf(
      "[WALL] %s | front=%.3fm left=%.3fm right=%.3fm\n",
      message,
      sensors.frontDistanceM,
      sensors.leftDistanceM,
      sensors.rightDistanceM);
}

void beginPhase(WallFollowPhase phase, uint32_t durationMs) {
  g_phase = phase;
  g_phaseStartMs = millis();
  g_phaseDurationMs = durationMs;

  switch (phase) {
    case WallFollowPhase::DriveForward:
      setMotorSpeeds(CAL_MOVE_SPEED, CAL_MOVE_SPEED);
      break;
    case WallFollowPhase::TurnLeft:
      setMotorSpeeds(-CAL_TURN_SPEED, CAL_TURN_SPEED);
      break;
    case WallFollowPhase::TurnRight:
      setMotorSpeeds(CAL_TURN_SPEED, -CAL_TURN_SPEED);
      break;
    case WallFollowPhase::TurnAround:
      setMotorSpeeds(CAL_TURN_SPEED, -CAL_TURN_SPEED);
      break;
    case WallFollowPhase::Settle:
    case WallFollowPhase::Stopped:
    case WallFollowPhase::Decide:
      stopMotors();
      break;
  }
}

void settleThen(WallFollowPhase nextPhase) {
  stopMotors();
  g_nextPhaseAfterSettle = nextPhase;
  beginPhase(WallFollowPhase::Settle, WALL_FOLLOW_SETTLE_MS);
}

bool phaseElapsed() {
  return millis() - g_phaseStartMs >= g_phaseDurationMs;
}

void driveForwardWithWallCorrection(const SensorReadings& sensors) {
  if (frontIsBlocked(sensors)) {
    settleThen(WallFollowPhase::Decide);
    return;
  }

  int leftCommand = CAL_MOVE_SPEED;
  int rightCommand = CAL_MOVE_SPEED;

  if (WALL_FOLLOW_LEFT_HAND && !leftIsOpen(sensors)) {
    // If left distance is larger than target, robot is too far from left wall:
    // slow left motor and speed right motor to arc left.
    const float errorM = sensors.leftDistanceM - WALL_FOLLOW_TARGET_SIDE_M;
    const int correction = clampCorrection(WALL_FOLLOW_KP * errorM);
    leftCommand = CAL_MOVE_SPEED - correction;
    rightCommand = CAL_MOVE_SPEED + correction;
  } else if (!WALL_FOLLOW_LEFT_HAND && !rightIsOpen(sensors)) {
    // If right distance is larger than target, robot is too far from right wall:
    // speed left motor and slow right motor to arc right.
    const float errorM = sensors.rightDistanceM - WALL_FOLLOW_TARGET_SIDE_M;
    const int correction = clampCorrection(WALL_FOLLOW_KP * errorM);
    leftCommand = CAL_MOVE_SPEED + correction;
    rightCommand = CAL_MOVE_SPEED - correction;
  }

  setMotorSpeeds(leftCommand, rightCommand);
}

void decideNextMove(const SensorReadings& sensors) {
  const bool frontOpen = frontIsOpen(sensors);
  const bool leftOpen = leftIsOpen(sensors);
  const bool rightOpen = rightIsOpen(sensors);

  if (WALL_FOLLOW_LEFT_HAND) {
    if (leftOpen) {
      logState("left open -> turn left", sensors);
      beginPhase(WallFollowPhase::TurnLeft, CAL_TURN_LEFT_90_MS);
    } else if (frontOpen) {
      logState("front open -> drive forward", sensors);
      beginPhase(WallFollowPhase::DriveForward, CAL_FORWARD_MS);
    } else if (rightOpen) {
      logState("right open -> turn right", sensors);
      beginPhase(WallFollowPhase::TurnRight, CAL_TURN_RIGHT_90_MS);
    } else {
      logState("dead end -> turn around", sensors);
      beginPhase(WallFollowPhase::TurnAround, WALL_FOLLOW_TURN_AROUND_MS);
    }
  } else {
    if (rightOpen) {
      logState("right open -> turn right", sensors);
      beginPhase(WallFollowPhase::TurnRight, CAL_TURN_RIGHT_90_MS);
    } else if (frontOpen) {
      logState("front open -> drive forward", sensors);
      beginPhase(WallFollowPhase::DriveForward, CAL_FORWARD_MS);
    } else if (leftOpen) {
      logState("left open -> turn left", sensors);
      beginPhase(WallFollowPhase::TurnLeft, CAL_TURN_LEFT_90_MS);
    } else {
      logState("dead end -> turn around", sensors);
      beginPhase(WallFollowPhase::TurnAround, WALL_FOLLOW_TURN_AROUND_MS);
    }
  }
}

}  // namespace

void initMazeSolver(MazeSolverState& state, MazeStrategy strategy) {
  state.strategy = strategy;
  state.initialized = true;

  g_state = state;
  g_phase = WallFollowPhase::Decide;
  g_nextPhaseAfterSettle = WallFollowPhase::Decide;
  g_phaseStartMs = millis();
  g_phaseDurationMs = 0;
  g_lastLogMs = 0;

  stopMotors();
  Serial.printf(
      "[WALL] initialized: %s-hand wall following\n",
      WALL_FOLLOW_LEFT_HAND ? "left" : "right");
}

void updateMazeSolver(const RobotPose& pose, const SensorReadings& sensors) {
  (void)pose;  // Pose/odometry is not needed for wall following.

  if (!g_state.initialized || g_state.strategy != MazeStrategy::WallFollowing) {
    stopMotors();
    return;
  }

  switch (g_phase) {
    case WallFollowPhase::Decide:
      decideNextMove(sensors);
      break;

    case WallFollowPhase::DriveForward:
      driveForwardWithWallCorrection(sensors);
      if (phaseElapsed()) {
        settleThen(WallFollowPhase::Decide);
      }
      break;

    case WallFollowPhase::TurnLeft:
    case WallFollowPhase::TurnRight:
    case WallFollowPhase::TurnAround:
      if (phaseElapsed()) {
        settleThen(WallFollowPhase::Decide);
      }
      break;

    case WallFollowPhase::Settle:
      if (phaseElapsed()) {
        beginPhase(g_nextPhaseAfterSettle, 0);
      }
      break;

    case WallFollowPhase::Stopped:
      stopMotors();
      break;
  }
}
