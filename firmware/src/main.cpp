#include <Arduino.h>
#include <WiFi.h>
#include <cstdarg>

#include "config.h"
#include "maze_solver.h"
#include "motor_driver.h"
#include "robot_state.h"
#include "sensors.h"

// Set to false when switching back to maze-solver behavior.
constexpr bool RUN_HARDWARE_SMOKE_TEST = false;

static RobotPose g_pose{};
static SensorReadings g_sensorReadings{};
static MazeSolverState g_solverState{};

namespace {
WiFiServer g_cmdServer(WIFI_CMD_PORT);
WiFiClient g_cmdClient;

void logLine(const char* line) {
  Serial.println(line);
  if (g_cmdClient && g_cmdClient.connected()) {
    g_cmdClient.println(line);
  }
}

void logf(const char* fmt, ...) {
  char buffer[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  logLine(buffer);
}

void connectWifiAndStartCommandServer() {
  if (WIFI_MODE == WifiMode::AccessPoint) {
    WiFi.mode(WIFI_AP);
    const bool started = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    if (!started) {
      logLine("[NET] failed to start AP; wireless commands disabled.");
      return;
    }
    logf("[NET] AP started: SSID='%s'", WIFI_AP_SSID);
    logf("[NET] AP IP=%s", WiFi.softAPIP().toString().c_str());
  } else {
    logf("[NET] connecting to SSID '%s' ...", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
      delay(250);
      ++attempts;
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
      logLine("[NET] Wi-Fi connect timeout; wireless commands disabled.");
      return;
    }
    logf("[NET] Wi-Fi connected, IP=%s", WiFi.localIP().toString().c_str());
  }

  g_cmdServer.begin();
  g_cmdServer.setNoDelay(true);
  logf("[NET] command server listening on TCP %u", WIFI_CMD_PORT);
}

void serviceCommandClient() {
  if (!g_cmdClient || !g_cmdClient.connected()) {
    WiFiClient incoming = g_cmdServer.available();
    if (incoming) {
      g_cmdClient.stop();
      g_cmdClient = incoming;
      g_cmdClient.setNoDelay(true);
      logf("[NET] client connected: %s", g_cmdClient.remoteIP().toString().c_str());
      g_cmdClient.println("[CTRL] connected. Type h for help.");
    }
  }
}

void runMotorPhase(const char* label, int left, int right, uint32_t durationMs) {
  logf("[MOTOR] %s | left=%d right=%d", label, left, right);
  setMotorSpeeds(left, right);
  delay(durationMs);
  stopMotors();
  delay(250);
}

void printManualControlHelp() {
  logLine("[CTRL] Commands:");
  logLine("[CTRL]   h            -> help");
  logLine("[CTRL]   f            -> forward calibrated distance");
  logLine("[CTRL]   b            -> reverse calibrated distance");
  logLine("[CTRL]   l            -> turn left 90 deg");
  logLine("[CTRL]   r            -> turn right 90 deg");
  logLine("[CTRL]   t            -> turn 360 deg (CW)");
  logLine("[CTRL]   p            -> toggle ToF printout");
  logLine("[CTRL]   s            -> stop motors");
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
      logf("[CTRL] ToF printout %s", tofPrintEnabled ? "enabled" : "disabled");
      break;
    case 's':
      stopMotors();
      logLine("[CTRL] stop");
      break;
    default:
      logf("[CTRL] Unknown command '%c'", command);
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

  while (g_cmdClient && g_cmdClient.connected() && g_cmdClient.available() > 0) {
    char c = static_cast<char>(g_cmdClient.read());
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
  serviceCommandClient();
  processSerialCommands(tofPrintEnabled);

  if (tofPrintEnabled && millis() - lastSensorPrintMs >= 200) {
    lastSensorPrintMs = millis();
    logf(
        "[TOF] front=%.3fm left=%.3fm right=%.3fm",
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

    logLine("[SMOKE] motor test sequence complete; continuing ToF stream + manual controls.");
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
    logLine("[SMOKE] Hardware smoke test mode enabled");
    logLine("[SMOKE] Open serial monitor @115200 to view ToF and motor phases");
    connectWifiAndStartCommandServer();
  } else {
    logLine("[MAZE] Wall-following mode enabled");
    initMazeSolver(g_solverState, MazeStrategy::WallFollowing);
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
