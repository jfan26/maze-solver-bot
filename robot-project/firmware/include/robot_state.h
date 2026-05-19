#pragma once

enum class MovementState {
  Idle,
  DrivingForward,
  TurningLeft,
  TurningRight,
  Braking
};

enum class CellOrientation {
  North,
  East,
  South,
  West
};

struct RobotPose {
  float xM;
  float yM;
  float headingRad;
  MovementState movement;
  CellOrientation orientation;
};

struct SensorReadings {
  float frontDistanceM;
  float leftDistanceM;
  float rightDistanceM;
};
