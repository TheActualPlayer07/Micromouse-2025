#ifndef FORWARD_PID_H
#define FORWARD_PID_H

#include <Arduino.h>

// PID variables
extern float speed_Kp;
extern float speed_Ki;
extern float speed_Kd;
extern float speed_integralA;
extern float speed_integralB;
extern float speed_prev_errorA;
extern float speed_prev_errorB;

// Target speed
extern float base_speed;
extern float current_target_speed;

// Output PWM
extern float pwmA_out;
extern float pwmB_out;

// Functions
void initPID();
void resetPID();
void updatePID(float targetSpeed, float currentSpeedA, float currentSpeedB);
float calculateTargetSpeed(uint16_t distance);
void smoothRamp(float desired_target);

#endif
