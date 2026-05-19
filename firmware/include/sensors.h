#pragma once

// Placeholder sensor interface for maze sensing.
// Intended for front/left/right distance sensing, likely via ToF sensors.
// Exact hardware and driver implementation is deferred.

void initSensors();
void updateSensors();

float getFrontDistance();
float getLeftDistance();
float getRightDistance();
