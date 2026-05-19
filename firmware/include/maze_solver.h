#pragma once

#include "robot_state.h"

enum class MazeStrategy {
  None,
  WallFollowing,
  FloodFill,
  DepthFirstSearch,
  BreadthFirstSearch
};

struct MazeSolverState {
  MazeStrategy strategy;
  bool initialized;
};

// TODO: Implement initialization and state updates for selected algorithm.
// Possible algorithms: wall following, flood fill, DFS, BFS.
void initMazeSolver(MazeSolverState& state, MazeStrategy strategy);
void updateMazeSolver(const RobotPose& pose, const SensorReadings& sensors);
