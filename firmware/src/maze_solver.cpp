#include "maze_solver.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "logging.h"
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
  OpeningPostTurnAdvance,
  Settle,
  Stopped
};

enum class DeferredOpeningTurn {
  None,
  Left,
  Right
};

MazeSolverState g_state{MazeStrategy::None, false};
WallFollowPhase g_phase = WallFollowPhase::Stopped;
WallFollowPhase g_nextPhaseAfterSettle = WallFollowPhase::Decide;
uint32_t g_phaseStartMs = 0;
uint32_t g_phaseDurationMs = 0;
uint32_t g_lastLogMs = 0;
uint32_t g_lastCorrectionLogMs = 0;
DeferredOpeningTurn g_deferredOpeningTurn = DeferredOpeningTurn::None;
uint32_t g_deferredOpeningTurnStartMs = 0;
bool g_postTurnAdvancePending = false;
uint8_t g_recoverySequenceCount = 0;
uint32_t g_lastRecoveryStartMs = 0;
uint32_t g_activeRecoveryBackupMs = WALL_FOLLOW_BACKUP_MS;
uint32_t g_activeRecoveryTurnMs = WALL_FOLLOW_RECOVERY_TURN_MS;
bool g_recoveryTurnLeft = WALL_FOLLOW_LEFT_HAND;
uint8_t g_leftOpenStreakTicks = 0;
uint8_t g_rightOpenStreakTicks = 0;

SensorReadings g_stuckReference{};
uint32_t g_stuckWindowStartMs = 0;
bool g_stuckReferenceValid = false;

float g_previousFollowDistanceM = 0.0f;
bool g_previousFollowDistanceValid = false;

struct WallCorrectionResult {
  float distanceErrorM;
  float distanceDeltaM;
  int rawCorrection;
};

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

bool sideReadingIndicatesOpening(float distanceM) {
  // A side opening should be a real large reading. Do not classify zero/NaN/
  // impossible readings as openings, because that can trigger false turns.
  return isFiniteUsableDistance(distanceM) && distanceM > WALL_FOLLOW_SIDE_OPEN_M;
}

bool sideReadingTracksNearbyWall(float distanceM) {
  return isFiniteUsableDistance(distanceM) && distanceM <= WALL_FOLLOW_SIDE_OPEN_M;
}

bool leftIsOpen(const SensorReadings& sensors) {
  return sideReadingIndicatesOpening(sensors.leftDistanceM);
}

bool rightIsOpen(const SensorReadings& sensors) {
  return sideReadingIndicatesOpening(sensors.rightDistanceM);
}

void beginPhase(WallFollowPhase phase, uint32_t durationMs);

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
  logf(
      "[WALL] %s | front=%.3fm left=%.3fm right=%.3fm",
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
  return isFiniteUsableDistance(distanceM);
}

bool stuckReferenceUsable(const SensorReadings& sensors) {
  // One usable sensor is enough to detect lack-of-motion. Requiring all three
  // lets a single bad reading (e.g. front sensor saturating against a close
  // wall) silently disable the whole recovery path.
  return distanceUsableForStuckDetection(sensors.frontDistanceM) ||
         distanceUsableForStuckDetection(sensors.leftDistanceM) ||
         distanceUsableForStuckDetection(sensors.rightDistanceM);
}

bool sensorReadingDeltaWithinTolerance(float current, float reference) {
  if (!isFiniteUsableDistance(current) || !isFiniteUsableDistance(reference)) {
    return true;
  }
  // Long-range readings have noise larger than STUCK_DELTA and would otherwise
  // reset the stuck timer continuously even when the robot is pinned.
  if (current > WALL_FOLLOW_STUCK_MAX_TRACKING_M &&
      reference > WALL_FOLLOW_STUCK_MAX_TRACKING_M) {
    return true;
  }
  return fabsf(current - reference) <= WALL_FOLLOW_STUCK_DELTA_M;
}

bool readingsStayedConstant(const SensorReadings& sensors, const SensorReadings& reference) {
  return sensorReadingDeltaWithinTolerance(sensors.frontDistanceM, reference.frontDistanceM) &&
         sensorReadingDeltaWithinTolerance(sensors.leftDistanceM, reference.leftDistanceM) &&
         sensorReadingDeltaWithinTolerance(sensors.rightDistanceM, reference.rightDistanceM);
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

void beginDriveForward(DeferredOpeningTurn deferredOpeningTurn) {
  g_deferredOpeningTurn = deferredOpeningTurn;
  g_deferredOpeningTurnStartMs =
      deferredOpeningTurn == DeferredOpeningTurn::None ? 0 : millis();
  g_postTurnAdvancePending = false;
  beginPhase(WallFollowPhase::DriveForward, 0);
}

void beginTurnWithOptionalPostAdvance(
    WallFollowPhase turnPhase,
    uint32_t durationMs,
    bool postTurnAdvancePending) {
  g_postTurnAdvancePending = postTurnAdvancePending;
  beginPhase(turnPhase, durationMs);
}

bool openingAdvanceActive() {
  return g_deferredOpeningTurn != DeferredOpeningTurn::None ||
         g_phase == WallFollowPhase::OpeningPostTurnAdvance;
}

int currentForwardBaseLeftSpeed() {
  return openingAdvanceActive() ? WALL_FOLLOW_FORWARD_LEFT_SPEED :
                                  WALL_FOLLOW_DRIVE_LEFT_SPEED;
}

int currentForwardBaseRightSpeed() {
  return openingAdvanceActive() ? WALL_FOLLOW_FORWARD_RIGHT_SPEED :
                                  WALL_FOLLOW_DRIVE_RIGHT_SPEED;
}

void beginPhase(WallFollowPhase phase, uint32_t durationMs) {
  g_phase = phase;
  g_phaseStartMs = millis();
  g_phaseDurationMs = durationMs;

  if (phase == WallFollowPhase::DriveForward ||
      phase == WallFollowPhase::OpeningPostTurnAdvance) {
    resetStuckDetector();
    resetWallCorrectionMemory();
    g_leftOpenStreakTicks = 0;
    g_rightOpenStreakTicks = 0;
  } else {
    g_deferredOpeningTurn = DeferredOpeningTurn::None;
    g_deferredOpeningTurnStartMs = 0;
  }

  switch (phase) {
    case WallFollowPhase::DriveForward:
      setMotorSpeeds(currentForwardBaseLeftSpeed(), currentForwardBaseRightSpeed());
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
      if (g_recoveryTurnLeft) {
        setMotorSpeeds(-WALL_FOLLOW_RECOVERY_TURN_SPEED, WALL_FOLLOW_RECOVERY_TURN_SPEED);
      } else {
        setMotorSpeeds(WALL_FOLLOW_RECOVERY_TURN_SPEED, -WALL_FOLLOW_RECOVERY_TURN_SPEED);
      }
      break;
    case WallFollowPhase::OpeningPostTurnAdvance:
      setMotorSpeeds(currentForwardBaseLeftSpeed(), currentForwardBaseRightSpeed());
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

uint32_t clampRecoveryDuration(uint32_t durationMs, uint32_t maxDurationMs) {
  return durationMs > maxDurationMs ? maxDurationMs : durationMs;
}

bool chooseRecoveryTurnDirection(const SensorReadings& sensors) {
  // Turn away from whichever side wall is closer; that's the side the robot is
  // pinned against. Falls back to the followed-wall direction if both side
  // readings can't be compared.
  const bool leftUsable = isFiniteUsableDistance(sensors.leftDistanceM);
  const bool rightUsable = isFiniteUsableDistance(sensors.rightDistanceM);
  if (leftUsable && rightUsable) {
    return sensors.rightDistanceM < sensors.leftDistanceM;
  }
  return WALL_FOLLOW_LEFT_HAND;
}

void beginStuckRecovery(const SensorReadings& sensors) {
  const uint32_t now = millis();
  if (g_recoverySequenceCount > 0 &&
      now - g_lastRecoveryStartMs <= WALL_FOLLOW_RECOVERY_STREAK_WINDOW_MS) {
    ++g_recoverySequenceCount;
  } else {
    g_recoverySequenceCount = 1;
  }
  g_lastRecoveryStartMs = now;

  const uint32_t escalationLevel = g_recoverySequenceCount - 1;
  g_activeRecoveryBackupMs = clampRecoveryDuration(
      WALL_FOLLOW_BACKUP_MS + escalationLevel * WALL_FOLLOW_RECOVERY_BACKUP_STEP_MS,
      WALL_FOLLOW_RECOVERY_BACKUP_MAX_MS);
  g_activeRecoveryTurnMs = clampRecoveryDuration(
      WALL_FOLLOW_RECOVERY_TURN_MS + escalationLevel * WALL_FOLLOW_RECOVERY_TURN_STEP_MS,
      WALL_FOLLOW_RECOVERY_TURN_MAX_MS);

  logState("steady sensor readings -> back up", sensors);
  logf(
      "[WALL] recovery streak=%u backup=%lums turn=%lums",
      static_cast<unsigned>(g_recoverySequenceCount),
      static_cast<unsigned long>(g_activeRecoveryBackupMs),
      static_cast<unsigned long>(g_activeRecoveryTurnMs));
  resetStuckDetector();
  resetWallCorrectionMemory();
  g_postTurnAdvancePending = false;
  beginPhase(WallFollowPhase::BackUp, g_activeRecoveryBackupMs);
}

bool deferredOpeningAdvanceElapsed() {
  return g_deferredOpeningTurn != DeferredOpeningTurn::None &&
         millis() - g_deferredOpeningTurnStartMs >= WALL_FOLLOW_OPENING_PRE_TURN_ADVANCE_MS;
}

float computeLeewayAdjustedDistanceError(float followDistanceM) {
  const float towardWallThresholdM = WALL_FOLLOW_TARGET_SIDE_M + WALL_FOLLOW_TOWARD_WALL_LEEWAY_M;
  const float awayFromWallThresholdM = WALL_FOLLOW_TARGET_SIDE_M - WALL_FOLLOW_TARGET_LEEWAY_M;

  if (followDistanceM > towardWallThresholdM) {
    return followDistanceM - towardWallThresholdM;
  }

  if (followDistanceM < awayFromWallThresholdM) {
    return followDistanceM - awayFromWallThresholdM;
  }

  return 0.0f;
}

WallCorrectionResult computeWallCorrection(float followDistanceM) {
  const float distanceErrorM = computeLeewayAdjustedDistanceError(followDistanceM);

  float distanceDeltaM = 0.0f;
  if (g_previousFollowDistanceValid) {
    distanceDeltaM = applyDeadband(
        followDistanceM - g_previousFollowDistanceM,
        WALL_FOLLOW_DISTANCE_DEADBAND_M);
  }

  g_previousFollowDistanceM = followDistanceM;
  g_previousFollowDistanceValid = true;

  return WallCorrectionResult{
      distanceErrorM,
      distanceDeltaM,
      clampCorrection(WALL_FOLLOW_KP * distanceErrorM + WALL_FOLLOW_KD * distanceDeltaM)};
}

int applyTowardWallCorrectionLimit(int correction, float distanceErrorM) {
  // Never steer farther into the followed wall if we are already at or inside
  // the target distance, even if the derivative term briefly swings positive.
  if (distanceErrorM <= 0.0f && correction > 0) {
    return 0;
  }

  if (correction > WALL_FOLLOW_MAX_TOWARD_WALL_CORRECTION) {
    return WALL_FOLLOW_MAX_TOWARD_WALL_CORRECTION;
  }
  return correction;
}

int scaleLeftHandCloseWallRightVeerCorrection(int correction, float distanceErrorM) {
  if (!WALL_FOLLOW_LEFT_HAND || distanceErrorM >= 0.0f || correction >= 0) {
    return correction;
  }

  return (correction * WALL_FOLLOW_LEFT_CLOSE_RIGHT_VEER_SCALE_PERCENT) / 100;
}

int keepForwardCommandAboveStaticFriction(int command) {
  if (command > 0 && command < WALL_FOLLOW_MIN_ACTIVE_FORWARD_SPEED) {
    return WALL_FOLLOW_MIN_ACTIVE_FORWARD_SPEED;
  }
  return command;
}

void applyForwardFloorPreservingDifferential(int& leftCommand, int& rightCommand) {
  // Floor-clamping one wheel without compensating the other silently cancels
  // the requested steering differential; boost the partner to preserve it.
  if (leftCommand <= 0 || rightCommand <= 0) {
    leftCommand = keepForwardCommandAboveStaticFriction(leftCommand);
    rightCommand = keepForwardCommandAboveStaticFriction(rightCommand);
    return;
  }
  const int floor = WALL_FOLLOW_MIN_ACTIVE_FORWARD_SPEED;
  if (leftCommand < floor) {
    rightCommand += floor - leftCommand;
    leftCommand = floor;
  }
  if (rightCommand < floor) {
    leftCommand += floor - rightCommand;
    rightCommand = floor;
  }
}

void maybeLogWallCorrection(
    const char* followedWallSide,
    float followDistanceM,
    const WallCorrectionResult& result,
    int appliedCorrection,
    int leftCommand,
    int rightCommand) {
  if (!WALL_FOLLOW_LOG_CORRECTIONS) {
    return;
  }

  const uint32_t now = millis();
  if (now - g_lastCorrectionLogMs < WALL_FOLLOW_LOG_PERIOD_MS) {
    return;
  }

  g_lastCorrectionLogMs = now;
  logf(
      "[WALL] steer side=%s dist=%.3fm target=%.3fm err=%.3fm delta=%.3fm raw=%d applied=%d left=%d right=%d",
      followedWallSide,
      followDistanceM,
      WALL_FOLLOW_TARGET_SIDE_M,
      result.distanceErrorM,
      result.distanceDeltaM,
      result.rawCorrection,
      appliedCorrection,
      leftCommand,
      rightCommand);
}

void applyContinuousWallCorrection(const SensorReadings& sensors) {
  const int baseLeftSpeed = currentForwardBaseLeftSpeed();
  const int baseRightSpeed = currentForwardBaseRightSpeed();
  int leftCommand = baseLeftSpeed;
  int rightCommand = baseRightSpeed;

  if (!WALL_FOLLOW_STEERING_ENABLED) {
    resetWallCorrectionMemory();
    applyForwardFloorPreservingDifferential(leftCommand, rightCommand);
    setMotorSpeeds(leftCommand, rightCommand);
    return;
  }

  if (WALL_FOLLOW_LEFT_HAND && sideReadingTracksNearbyWall(sensors.leftDistanceM)) {
    // Positive correction arcs left; negative correction arcs right.
    // If the left reading is increasing, the robot is drifting away from the
    // left wall, so arc left. If it is decreasing, arc right.
    const WallCorrectionResult result = computeWallCorrection(sensors.leftDistanceM);
    const int correction = scaleLeftHandCloseWallRightVeerCorrection(
        applyTowardWallCorrectionLimit(result.rawCorrection, result.distanceErrorM),
        result.distanceErrorM);
    leftCommand = baseLeftSpeed - correction;
    rightCommand = baseRightSpeed + correction;
    applyForwardFloorPreservingDifferential(leftCommand, rightCommand);
    maybeLogWallCorrection("left", sensors.leftDistanceM, result, correction, leftCommand, rightCommand);
  } else if (!WALL_FOLLOW_LEFT_HAND && sideReadingTracksNearbyWall(sensors.rightDistanceM)) {
    // Positive correction arcs right; negative correction arcs left.
    const WallCorrectionResult result = computeWallCorrection(sensors.rightDistanceM);
    const int correction = applyTowardWallCorrectionLimit(result.rawCorrection, result.distanceErrorM);
    leftCommand = baseLeftSpeed + correction;
    rightCommand = baseRightSpeed - correction;
    applyForwardFloorPreservingDifferential(leftCommand, rightCommand);
    maybeLogWallCorrection("right", sensors.rightDistanceM, result, correction, leftCommand, rightCommand);
  } else {
    resetWallCorrectionMemory();
    applyForwardFloorPreservingDifferential(leftCommand, rightCommand);
  }

  setMotorSpeeds(leftCommand, rightCommand);
}

void decideNextMove(const SensorReadings& sensors) {
  const bool frontOpen = frontIsOpen(sensors);
  const bool leftOpen = leftIsOpen(sensors);
  const bool rightOpen = rightIsOpen(sensors);

  if (WALL_FOLLOW_LEFT_HAND) {
    if (leftOpen) {
      if (frontOpen) {
        logState("left open + front open -> timed advance before turn left", sensors);
        beginDriveForward(DeferredOpeningTurn::Left);
      } else {
        logState("left open -> turn left", sensors);
        beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnLeft, CAL_TURN_LEFT_90_MS, false);
      }
    } else if (frontOpen) {
      logState("front open -> continuous drive forward", sensors);
      beginDriveForward(DeferredOpeningTurn::None);
    } else if (rightOpen) {
      logState("right open -> turn right", sensors);
      beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnRight, CAL_TURN_RIGHT_90_MS, false);
    } else {
      logState("dead end -> turn around", sensors);
      beginPhase(WallFollowPhase::TurnAround, WALL_FOLLOW_TURN_AROUND_MS);
    }
  } else {
    if (rightOpen) {
      if (frontOpen) {
        logState("right open + front open -> timed advance before turn right", sensors);
        beginDriveForward(DeferredOpeningTurn::Right);
      } else {
        logState("right open -> turn right", sensors);
        beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnRight, CAL_TURN_RIGHT_90_MS, false);
      }
    } else if (frontOpen) {
      logState("front open -> continuous drive forward", sensors);
      beginDriveForward(DeferredOpeningTurn::None);
    } else if (leftOpen) {
      logState("left open -> turn left", sensors);
      beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnLeft, CAL_TURN_LEFT_90_MS, false);
    } else {
      logState("dead end -> turn around", sensors);
      beginPhase(WallFollowPhase::TurnAround, WALL_FOLLOW_TURN_AROUND_MS);
    }
  }
}

void updateOpeningStreakTicks(const SensorReadings& sensors) {
  if (leftIsOpen(sensors)) {
    if (g_leftOpenStreakTicks < 255) ++g_leftOpenStreakTicks;
  } else {
    g_leftOpenStreakTicks = 0;
  }
  if (rightIsOpen(sensors)) {
    if (g_rightOpenStreakTicks < 255) ++g_rightOpenStreakTicks;
  } else {
    g_rightOpenStreakTicks = 0;
  }
}

bool leftOpeningStreakConfirmed() {
  return g_leftOpenStreakTicks >= WALL_FOLLOW_OPENING_DEBOUNCE_TICKS;
}

bool rightOpeningStreakConfirmed() {
  return g_rightOpenStreakTicks >= WALL_FOLLOW_OPENING_DEBOUNCE_TICKS;
}

void driveForwardContinuously(const SensorReadings& sensors) {
  // Preserve the wall-following priority while driving continuously. This lets
  // the robot react as soon as it sees a side opening or front obstruction.
  updateOpeningStreakTicks(sensors);

  if (g_deferredOpeningTurn == DeferredOpeningTurn::Left) {
    if (deferredOpeningAdvanceElapsed()) {
      logState("timed advance done -> delayed left turn", sensors);
      beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnLeft, CAL_TURN_LEFT_90_MS, true);
      return;
    }

    if (frontIsBlocked(sensors)) {
      logState("front blocked during timed advance -> turn left now", sensors);
      beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnLeft, CAL_TURN_LEFT_90_MS, true);
      return;
    }

    if (stuckDetectedWhileDriving(sensors)) {
      beginStuckRecovery(sensors);
      return;
    }

    applyContinuousWallCorrection(sensors);
    return;
  }

  if (g_deferredOpeningTurn == DeferredOpeningTurn::Right) {
    if (deferredOpeningAdvanceElapsed()) {
      logState("timed advance done -> delayed right turn", sensors);
      beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnRight, CAL_TURN_RIGHT_90_MS, true);
      return;
    }

    if (frontIsBlocked(sensors)) {
      logState("front blocked during timed advance -> turn right now", sensors);
      beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnRight, CAL_TURN_RIGHT_90_MS, true);
      return;
    }

    if (stuckDetectedWhileDriving(sensors)) {
      beginStuckRecovery(sensors);
      return;
    }

    applyContinuousWallCorrection(sensors);
    return;
  }

  if (WALL_FOLLOW_LEFT_HAND && leftOpeningStreakConfirmed()) {
    if (frontIsOpen(sensors)) {
      logState("left opening confirmed while driving -> timed advance before turn left", sensors);
      g_deferredOpeningTurn = DeferredOpeningTurn::Left;
      g_deferredOpeningTurnStartMs = millis();
    } else {
      logState("left opening confirmed while driving -> turn left", sensors);
      beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnLeft, CAL_TURN_LEFT_90_MS, false);
      return;
    }
  }

  if (!WALL_FOLLOW_LEFT_HAND && rightOpeningStreakConfirmed()) {
    if (frontIsOpen(sensors)) {
      logState("right opening confirmed while driving -> timed advance before turn right", sensors);
      g_deferredOpeningTurn = DeferredOpeningTurn::Right;
      g_deferredOpeningTurnStartMs = millis();
    } else {
      logState("right opening confirmed while driving -> turn right", sensors);
      beginTurnWithOptionalPostAdvance(WallFollowPhase::TurnRight, CAL_TURN_RIGHT_90_MS, false);
      return;
    }
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
  g_lastCorrectionLogMs = 0;
  g_deferredOpeningTurn = DeferredOpeningTurn::None;
  g_deferredOpeningTurnStartMs = 0;
  g_postTurnAdvancePending = false;
  g_recoverySequenceCount = 0;
  g_lastRecoveryStartMs = 0;
  g_activeRecoveryBackupMs = WALL_FOLLOW_BACKUP_MS;
  g_activeRecoveryTurnMs = WALL_FOLLOW_RECOVERY_TURN_MS;
  g_recoveryTurnLeft = WALL_FOLLOW_LEFT_HAND;
  g_leftOpenStreakTicks = 0;
  g_rightOpenStreakTicks = 0;
  resetStuckDetector();
  resetWallCorrectionMemory();

  stopMotors();
  logf(
      "[WALL] initialized: %s-hand wall following, continuous drive enabled",
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
      if (phaseElapsed()) {
        if (g_postTurnAdvancePending) {
          g_postTurnAdvancePending = false;
          logState("turn done -> timed post-turn advance", sensors);
          beginPhase(WallFollowPhase::OpeningPostTurnAdvance, WALL_FOLLOW_OPENING_POST_TURN_ADVANCE_MS);
        } else {
          settleThen(WallFollowPhase::Decide);
        }
      }
      break;

    case WallFollowPhase::TurnAround:
      if (phaseElapsed()) {
        settleThen(WallFollowPhase::Decide);
      }
      break;

    case WallFollowPhase::BackUp:
      if (phaseElapsed()) {
        g_recoveryTurnLeft = chooseRecoveryTurnDirection(sensors);
        logState(
            g_recoveryTurnLeft ? "backup done -> counter-clockwise recovery turn" :
                                 "backup done -> clockwise recovery turn",
            sensors);
        beginPhase(WallFollowPhase::RecoveryTurnAway, g_activeRecoveryTurnMs);
      }
      break;

    case WallFollowPhase::RecoveryTurnAway:
      if (phaseElapsed()) {
        logState("recovery turn done -> re-evaluate", sensors);
        settleThen(WallFollowPhase::Decide);
      }
      break;

    case WallFollowPhase::OpeningPostTurnAdvance:
      if (frontIsBlocked(sensors)) {
        logState("post-turn advance front blocked -> re-evaluate", sensors);
        settleThen(WallFollowPhase::Decide);
      } else if (stuckDetectedWhileDriving(sensors)) {
        beginStuckRecovery(sensors);
      } else if (phaseElapsed()) {
        logState("post-turn advance done -> re-evaluate", sensors);
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
