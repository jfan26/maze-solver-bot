#include "sensors.h"

#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "config.h"

namespace {
VL53L0X frontSensor;
VL53L0X leftSensor;
VL53L0X rightSensor;

uint16_t frontRangeMm = 8190;
uint16_t leftRangeMm = 8190;
uint16_t rightRangeMm = 8190;

bool initSingleSensor(VL53L0X& sensor, uint8_t xshutPin, uint8_t addr, const char* name) {
  digitalWrite(xshutPin, HIGH);
  delay(10);

  if (!sensor.init()) {
    Serial.printf("[ToF] %s init failed\n", name);
    return false;
  }

  sensor.setAddress(addr);
  sensor.startContinuous(TOF_CONTINUOUS_PERIOD_MS);
  Serial.printf("[ToF] %s initialized at 0x%02X\n", name, addr);
  return true;
}
}  // namespace

void initSensors() {
  pinMode(TOF_FRONT_XSHUT, OUTPUT);
  pinMode(TOF_LEFT_XSHUT, OUTPUT);
  pinMode(TOF_RIGHT_XSHUT, OUTPUT);

  // 1) Hold all XSHUT low so all sensors stay in reset.
  digitalWrite(TOF_FRONT_XSHUT, LOW);
  digitalWrite(TOF_LEFT_XSHUT, LOW);
  digitalWrite(TOF_RIGHT_XSHUT, LOW);
  delay(10);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  // 2) Front only -> 0x30
  initSingleSensor(frontSensor, TOF_FRONT_XSHUT, TOF_FRONT_ADDR, "front");

  // 3) Left only -> 0x31
  initSingleSensor(leftSensor, TOF_LEFT_XSHUT, TOF_LEFT_ADDR, "left");

  // 4) Right only -> 0x32
  initSingleSensor(rightSensor, TOF_RIGHT_XSHUT, TOF_RIGHT_ADDR, "right");
}

void updateSensors() {
  frontRangeMm = frontSensor.readRangeContinuousMillimeters();
  leftRangeMm = leftSensor.readRangeContinuousMillimeters();
  rightRangeMm = rightSensor.readRangeContinuousMillimeters();
}

float getFrontDistance() { return frontRangeMm / 1000.0f; }
float getLeftDistance() { return leftRangeMm / 1000.0f; }
float getRightDistance() { return rightRangeMm / 1000.0f; }
