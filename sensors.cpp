#include "sensors.h"

const int XSHUT_FRONT = 25;
const int XSHUT_LEFT = 24;
const int XSHUT_RIGHT = 26;

VL53L0X frontSensor;
VL53L0X leftSensor;
VL53L0X rightSensor;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void initSensors() {
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);
  
  Wire.begin();
  
  Serial.println("Init BNO055 IMU");
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 failed!");
    while(1);
  }
  delay(100);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 OK");
  
  Serial.println("Init left sensor");
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  leftSensor.setTimeout(500);
  if (!leftSensor.init()) {
    Serial.println("ERROR: Left sensor failed!");
    while(1);
  }
  leftSensor.setAddress(0x30);
  Serial.println("Left OK");
  
  Serial.println("Init front sensor");
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);
  frontSensor.setTimeout(500);
  if (!frontSensor.init()) {
    Serial.println("ERROR: Front sensor failed!");
    while(1);
  }
  frontSensor.setAddress(0x31);
  Serial.println("Front OK");
  
  Serial.println("Init right sensor");
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  rightSensor.setTimeout(500);
  if (!rightSensor.init()) {
    Serial.println("ERROR: Right sensor failed!");
    while(1);
  }
  Serial.println("Right OK");
}

uint16_t readFront() {
  uint16_t distance = frontSensor.readRangeSingleMillimeters();
  return frontSensor.timeoutOccurred() ? 2000 : distance;
}

uint16_t readLeft() {
  uint16_t distance = leftSensor.readRangeSingleMillimeters();
  return leftSensor.timeoutOccurred() ? 2000 : distance;
}

uint16_t readRight() {
  uint16_t distance = rightSensor.readRangeSingleMillimeters();
  return rightSensor.timeoutOccurred() ? 2000 : distance;
}

float getYaw() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x; 
}
