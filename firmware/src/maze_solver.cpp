#include "maze_solver.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "motor_driver.h"

namespace {

// This implementation uses a simple left-hand or right-hand wall-following rule.
// Forward motion is continuous: once it chooses to drive forward, it keeps
// updating motor speeds every control loop from the side ToF reading until it
// sees a wall-following decision point, front obstruction, or stuck condition.

enum class WallFollowPhase {
  Decide,
  DriveForward,
  TurnLeft,
  TurnRight,
  TurnAround,
  BackUp,
  RecoveryTurnAway,
  Settle,
  Stopped
};

MazeSolverState g_state{MazeStrategy::None, false};
WallFollowPhase g_phase = WallFollowPhase::Stopped;
WallFollowPhase g_nextPhaseAfterSettle = WallFollowPhase::Decide;
uint32_t g_phaseStartMs = 0;
uint32_t g_phaseDurationMs = 0;
uint32_t g_lastLogMs = 0;

SensorReadings g_stuckReference{};
uint32_t g_stuckWindowStartMs = 0;
bool g_stuckReferenceValid = false;

float g_previousFollowDistanceM = 0.0f;
bool g_previousFollowDistanceValid = false;

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

float applyDeadband(float value, float deadband) {
  if (fabsf(value) < deadband) {
    return 0.0f;
  }
  return value;
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

void resetStuckDetector() {
  g_stuckReference = SensorReadings{};
  g_stuckWindowStartMs = millis();
  g_stuckReferenceValid = false;
}

void resetWallCorrectionMemory() {
  g_previousFollowDistanceM = 0.0f;
  g_previousFollowDistanceValid = false;
}

bool distanceUsableForStuckDetection(float distanceM) {
  return isFiniteUsableDistance(distanceM) && distanceM < WALL_FOLLOW_STUCK_MAX_SENSOR_M;
}

bool stuckReferenceUsable(const SensorReadings& sensors) {
  return distanceUsableForStuckDetection(sensors.frontDistanceM) &&
         distanceUsableForStuckDetection(sensors.leftDistanceM) &&
         distanceUsableForStuckDetection(sensors.rightDistanceM);
}

bool readingsStayedConstant(const SensorReadings& sensors, const SensorReadings& reference) {
  return fabsf(sensors.frontDistanceM - reference.frontDistanceM) <= WALL_FOLLOW_STUCK_DELTA_M &&
         fabsf(sensors.leftDistanceM - reference.leftDistanceM) <= WALL_FOLLOW_STUCK_DELTA_M &&
         fabsf(sensors.rightDistanceM - reference.rightDistanceM) <= WALL_FOLLOW_STUCK_DELTA_M;
}

bool stuckDetectedWhileDriving(const SensorReadings& sensors) {
  if (!stuckReferenceUsable(sensors)) {
    resetStuckDetector();
    return false;
  }

  const uint32_t now = millis();

  if (!g_stuckReferenceValid) {
    g_stuckReference = sensors;
    g_stuckWindowStartMs = now;
    g_stuckReferenceValid = true;
    return false;
  }

  if (!readingsStayedConstant(sensors, g_stuckReference)) {
    g_stuckReference = sensors;
    g_stuckWindowStartMs = now;
    return false;
  }

  return now - g_stuckWindowStartMs >= WALL_FOLLOW_STUCK_MS;
}

void beginPhase(WallFollowPhase phase, uint32_t durationMs) {
  g_phase = phase;
  g_phaseStartMs = millis();
  g_phaseDurationMs = durationMs;

  if (phase == WallFollowPhase::DriveForward) {
    resetStuckDetector();
    resetWallCorrectionMemory();
  }

  switch (phase) {
    case WallFollowPhase::DriveForward:
      setMotorSpeeds(CAL_FORWARD_LEFT_SPEED, CAL_FORWARD_RIGHT_SPEED);
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
    case WallFollowPhase::BackUp:
      setMotorSpeeds(-WALL_FOLLOW_BACKUP_SPEED, -WALL_FOLLOW_BACKUP_SPEED);
      break;
    case WallFollowPhase::RecoveryTurnAway:
      if (WALL_FOLLOW_LEFT_HAND) {
        // Left-hand following: assume we scraped/pinned the left wall, so turn
        // slightly clockwise/right to point away from it.
        setMotorSpeeds(WALL_FOLLOW_RECOVERY_TURN_SPEED, -WALL_FOLLOW_RECOVERY_TURN_SPEED);
      } else {
        // Right-hand following: turn slightly counter-clockwise/left.
        setMotorSpeeds(-WALL_FOLLOW_RECOVERY_TURN_SPEED, WALL_FOLLOW_RECOVERY_TURN_SPEED);
      }
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

void beginStuckRecovery(const SensorReadings& sensors) {
  logState("stuck/collision suspected -> back up", sensors);
  resetStuckDetector();
  resetWallCorrectionMemory();
  beginPhase(WallFollowPhase::BackUp, WALL_FOLLOW_BACKUP_MS);
}

int computeWallCorrection(float followDistanceM) {
  const float distanceErrorM = applyDeadband(
      followDistanceM - WALL_FOLLOW_TARGET_SIDE_M,
      WALL_FOLLOW_DISTANCE_DEADBAND_M);

  float distanceDeltaM = 0.0f;
  if (g_previousFollowDistanceValid) {
    distanceDeltaM = applyDeadband(
        followDistanceM - g_previousFollowDistanceM,
        WALL_FOLLOW_DISTANCE_DEADBAND_M);
  }

  g_previousFollowDistanceM = followDistanceM;
  g_previousFollowDistanceValid = true;

  return clampCorrection(WALL_FOLLOW_KP * distanceErrorM + WALL_FOLLOW_KD * distanceDeltaM);
}

void applyContinuousWallCorrection(const SensorReadings& sensors) {
  int leftCommand = CAL_FORWARD_LEFT_SPEED;
  int rightCommand = CAL_FORWARD_RIGHT_SPEED;

  if (WALL_FOLLOW_LEFT_HAND && !leftIsOpen(sensors)) {
    // Positive correction arcs left; negative correction arcs right.
    // If the left reading is increasing, the robot is drifting away from the
    // left wall, so arc left. If it is decreasing, arc right.
    const int correction = computeWallCorrection(sensors.leftDistanceM);
    leftCommand = CAL_FORWARD_LEFT_SPEED - correction;
    rightCommand = CAL_FORWARD_RIGHT_SPEED + correction;
  } else if (!WALL_FOLLOW_LEFT_HAND && !rightIsOpen(sensors)) {
    // Positive correction arcs right; negative correction arcs left.
    const int correction = computeWallCorrection(sensors.rightDistanceM);
    leftCommand = CAL_FORWARD_LEFT_SPEED + correction;
    rightCommand = CAL_FORWARD_RIGHT_SPEED - correction;
  } else {
    resetWallCorrectionMemory();
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
      logState("front open -> continuous drive forward", sensors);
      beginPhase(WallFollowPhase::DriveForward, 0);
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
      logState("front open -> continuous drive forward", sensors);
      beginPhase(WallFollowPhase::DriveForward, 0);
    } else if (leftOpen) {
      logState("left open -> turn left", sensors);
      beginPhase(WallFollowPhase::TurnLeft, CAL_TURN_LEFT_90_MS);
    } else {
      logState("dead end -> turn around", sensors);
      beginPhase(WallFollowPhase::TurnAround, WALL_FOLLOW_TURN_AROUND_MS);
    }
  }
}

void driveForwardContinuously(const SensorReadings& sensors) {
  // Preserve the wall-following priority while driving continuously. This lets
  // the robot react as soon as it sees a side opening or front obstruction.
  if (WALL_FOLLOW_LEFT_HAND && leftIsOpen(sensors)) {
    logState("left opening detected while driving -> turn left", sensors);
    beginPhase(WallFollowPhase::TurnLeft, CAL_TURN_LEFT_90_MS);
    return;
  }

  if (!WALL_FOLLOW_LEFT_HAND && rightIsOpen(sensors)) {
    logState("right opening detected while driving -> turn right", sensors);
    beginPhase(WallFollowPhase::TurnRight, CAL_TURN_RIGHT_90_MS);
    return;
  }

  if (frontIsBlocked(sensors)) {
    logState("front blocked while driving -> decide", sensors);
    decideNextMove(sensors);
    return;
  }

  if (stuckDetectedWhileDriving(sensors)) {
    beginStuckRecovery(sensors);
    return;
  }

  applyContinuousWallCorrection(sensors);
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
  resetStuckDetector();
  resetWallCorrectionMemory();

  stopMotors();
  Serial.printf(
      "[WALL] initialized: %s-hand wall following, continuous drive enabled\n",
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
      driveForwardContinuously(sensors);
      break;

    case WallFollowPhase::TurnLeft:
    case WallFollowPhase::TurnRight:
    case WallFollowPhase::TurnAround:
      if (phaseElapsed()) {
        settleThen(WallFollowPhase::Decide);
      }
      break;

    case WallFollowPhase::BackUp:
      if (phaseElapsed()) {
        logState(
            WALL_FOLLOW_LEFT_HAND ? "backup done -> small clockwise correction" :
                                    "backup done -> small counter-clockwise correction",
            sensors);
        beginPhase(WallFollowPhase::RecoveryTurnAway, WALL_FOLLOW_RECOVERY_TURN_MS);
      }
      break;

    case WallFollowPhase::RecoveryTurnAway:
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
