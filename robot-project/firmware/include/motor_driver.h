#pragma once

// Placeholder motor interface for two-wheel differential drive.
// Positive values should command forward wheel motion.
// Negative values should command reverse wheel motion.
// Implementation details will depend on the chosen motor driver hardware.

void initMotors();
void setMotorSpeeds(float left, float right);
void stopMotors();
