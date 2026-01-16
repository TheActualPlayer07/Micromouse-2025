#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Sensor objects
extern VL53L0X frontSensor;
extern VL53L0X leftSensor;
extern VL53L0X rightSensor;
extern Adafruit_BNO055 bno;

// Functions
void initSensors();
uint16_t readFront();
uint16_t readLeft();
uint16_t readRight();
float getYaw();

#endif
