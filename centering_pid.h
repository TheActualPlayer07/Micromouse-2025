#ifndef CENTERING_PID_H
#define CENTERING_PID_H

#include <Arduino.h>

extern float centre_Kp;
extern float centre_Ki;
extern float centre_Kd;
extern float centre_integral;
extern float centre_prev_error;
extern unsigned long centre_prev_time;

void initCenteringPID();
void resetCenteringPID();
float computeCenteringCorrection(uint16_t left_dist, uint16_t right_dist);

#endif
