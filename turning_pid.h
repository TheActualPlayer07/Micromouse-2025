#ifndef TURNING_PID_H
#define TURNING_PID_H

#include <Arduino.h>

// PID gains for turning
extern float turn_Kp;
extern float turn_Ki;
extern float turn_Kd;

// Functions
void initTurningPID();
void resetTurningPID();
void turnToAngle(float target_angle, float current_angle);
float normalizeAngle(float angle);
float getAngleDifference(float target, float current);

#endif
