#include <Arduino.h>
#include <WiFi.h>
#include <cstdarg>

#include "config.h"
#include "maze_solver.h"
#include "motor_driver.h"
#include "robot_state.h"
#include "sensors.h"

// Set to false to skip automatic smoke-test sequence at boot.
constexpr bool RUN_HARDWARE_SMOKE_TEST_AT_BOOT = true;

static RobotPose g_pose{};
static SensorReadings g_sensorReadings{};
static MazeSolverState g_solverState{};

namespace {
WiFiServer g_cmdServer(WIFI_CMD_PORT);
WiFiClient g_cmdClient;

enum class ControlMode { Manual, MazeSolver };
ControlMode g_controlMode = ControlMode::Manual;

struct TimedMotorAction {
  bool active;
  int left;
  int right;
  uint32_t startedAtMs;
  uint32_t durationMs;
  const char* label;
};

TimedMotorAction g_timedAction{false, 0, 0, 0, 0, nullptr};
bool g_tofPrintEnabled = true;
bool g_smokeSequenceDone = false;
uint32_t g_lastSensorPrintMs = 0;
uint32_t g_lastSequenceStartMs = 0;

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

void stopTimedMotorAction() {
  if (g_timedAction.active) {
    logf("[MOTOR] interrupted '%s'", g_timedAction.label ? g_timedAction.label : "action");
  }
  g_timedAction.active = false;
  stopMotors();
}

void startTimedMotorAction(const char* label, int left, int right, uint32_t durationMs) {
  g_timedAction.active = true;
  g_timedAction.left = left;
  g_timedAction.right = right;
  g_timedAction.startedAtMs = millis();
  g_timedAction.durationMs = durationMs;
  g_timedAction.label = label;

  logf("[MOTOR] %s | left=%d right=%d duration=%lums", label, left, right, durationMs);
  setMotorSpeeds(left, right);
}

void serviceTimedMotorAction() {
  if (!g_timedAction.active) {
    return;
  }

  if (millis() - g_timedAction.startedAtMs >= g_timedAction.durationMs) {
    logf("[MOTOR] completed '%s'", g_timedAction.label ? g_timedAction.label : "action");
    g_timedAction.active = false;
    stopMotors();
  }
}

void enterMazeSolverMode() {
  stopTimedMotorAction();
  initMazeSolver(g_solverState, MazeStrategy::WallFollowing);
  g_controlMode = ControlMode::MazeSolver;
  logLine("[MODE] maze solver started (wall-following)");
}

void stopMazeSolverMode() {
  g_solverState.initialized = false;
  g_controlMode = ControlMode::Manual;
  stopMotors();
  logLine("[MODE] maze solver stopped; manual control enabled");
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

void printManualControlHelp() {
  logLine("[CTRL] Commands:");
  logLine("[CTRL]   h            -> help");
  logLine("[CTRL]   f            -> forward calibrated distance");
  logLine("[CTRL]   b            -> reverse calibrated distance");
  logLine("[CTRL]   l            -> turn left 30 deg");
  logLine("[CTRL]   r            -> turn right 30 deg");
  logLine("[CTRL]   t            -> turn 360 deg (CW)");
  logLine("[CTRL]   p            -> toggle ToF printout");
  logLine("[CTRL]   m            -> start maze solver mode");
  logLine("[CTRL]   x            -> stop maze solver mode");
  logLine("[CTRL]   s            -> stop current motion immediately");
}

void executeManualCommand(char command) {
  switch (command) {
    case 'h':
      printManualControlHelp();
      break;
    case 'f':
      startTimedMotorAction("forward calibrated", CAL_MOVE_SPEED, CAL_MOVE_SPEED, CAL_FORWARD_MS);
      break;
    case 'b':
      startTimedMotorAction("reverse calibrated", -CAL_MOVE_SPEED, -CAL_MOVE_SPEED, CAL_FORWARD_MS);
      break;
    case 'l':
      startTimedMotorAction("turn left 30", -CAL_TURN_SPEED, CAL_TURN_SPEED, CAL_TURN_LEFT_30_MS);
      break;
    case 'r':
      startTimedMotorAction("turn right 30", CAL_TURN_SPEED, -CAL_TURN_SPEED, CAL_TURN_RIGHT_30_MS);
      break;
    case 't':
      startTimedMotorAction("turn 360", CAL_TURN_SPEED, -CAL_TURN_SPEED, CAL_TURN_360_MS);
      break;
    case 'p':
      g_tofPrintEnabled = !g_tofPrintEnabled;
      logf("[CTRL] ToF printout %s", g_tofPrintEnabled ? "enabled" : "disabled");
      break;
    case 'm':
      enterMazeSolverMode();
      break;
    case 'x':
      stopMazeSolverMode();
      break;
    case 's':
      stopTimedMotorAction();
      stopMazeSolverMode();
      logLine("[CTRL] stop");
      break;
    default:
      logf("[CTRL] Unknown command '%c'", command);
      printManualControlHelp();
      break;
  }
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r' || c == ' ') {
      continue;
    }
    executeManualCommand(c);
  }

  while (g_cmdClient && g_cmdClient.connected() && g_cmdClient.available() > 0) {
    char c = static_cast<char>(g_cmdClient.read());
    if (c == '\n' || c == '\r' || c == ' ') {
      continue;
    }
    executeManualCommand(c);
  }
}

void serviceSmokeTestSequence() {
  if (g_smokeSequenceDone || !RUN_HARDWARE_SMOKE_TEST_AT_BOOT || g_controlMode != ControlMode::Manual) {
    return;
  }

  if (millis() - g_lastSequenceStartMs <= 1500 || g_timedAction.active) {
    return;
  }

  static uint8_t phase = 0;
  constexpr int testSpeed = 120;
  constexpr uint32_t phaseMs = 700;

  switch (phase) {
    case 0: startTimedMotorAction("left-forward", testSpeed, 0, phaseMs); break;
    case 1: startTimedMotorAction("left-reverse", -testSpeed, 0, phaseMs); break;
    case 2: startTimedMotorAction("right-forward", 0, testSpeed, phaseMs); break;
    case 3: startTimedMotorAction("right-reverse", 0, -testSpeed, phaseMs); break;
    case 4: startTimedMotorAction("both-forward", testSpeed, testSpeed, phaseMs); break;
    case 5: startTimedMotorAction("both-reverse", -testSpeed, -testSpeed, phaseMs); break;
    default:
      g_smokeSequenceDone = true;
      logLine("[SMOKE] motor test sequence complete; continuing ToF stream + manual controls.");
      printManualControlHelp();
      return;
  }
  ++phase;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("[BOOT] init motors/sensors");
  initMotors();
  initSensors();
  connectWifiAndStartCommandServer();

  g_controlMode = ControlMode::Manual;
  logLine("[MODE] manual control mode enabled");
  if (RUN_HARDWARE_SMOKE_TEST_AT_BOOT) {
    logLine("[SMOKE] Hardware smoke test mode enabled");
    logLine("[SMOKE] Open serial monitor @115200 to view ToF and motor phases");
  }
  printManualControlHelp();
}

void loop() {
  updateSensors();
  serviceCommandClient();
  processSerialCommands();
  serviceTimedMotorAction();

  if (g_tofPrintEnabled && millis() - g_lastSensorPrintMs >= 200) {
    g_lastSensorPrintMs = millis();
    logf("[TOF] front=%.3fm left=%.3fm right=%.3fm", getFrontDistance(), getLeftDistance(), getRightDistance());
  }

  serviceSmokeTestSequence();

  if (g_controlMode == ControlMode::MazeSolver) {
    g_sensorReadings.frontDistanceM = getFrontDistance();
    g_sensorReadings.leftDistanceM = getLeftDistance();
    g_sensorReadings.rightDistanceM = getRightDistance();
    updateMazeSolver(g_pose, g_sensorReadings);
  }

  delay(CONTROL_LOOP_PERIOD_MS);
}
