#include "centering_pid.h"

float centre_Kp = 0.15;
float centre_Ki = 0.002; 
float centre_Kd = 0.03;
float centre_prev_error = 0;
float centre_integral = 0;
float filtered_error = 0;
unsigned long centre_prev_time = 0;

const float CENTERING_DEADBAND = 3.0;
const float FILTER_ALPHA = 0.3;

void initCenteringPID() {
  resetCenteringPID();
}

void resetCenteringPID() {
  centre_prev_error = 0;
  centre_integral = 0;
  filtered_error = 0;
  centre_prev_time = millis();
}

float computeCenteringCorrection(uint16_t left_dist, uint16_t right_dist) {
  
  unsigned long current_time = millis();
  float dt = (current_time - centre_prev_time) / 1000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.02;
  centre_prev_time = current_time;
  
  float raw_error = (float)(left_dist - right_dist);
  filtered_error = (FILTER_ALPHA * raw_error) + ((1.0 - FILTER_ALPHA) * filtered_error);
  
  if (abs(filtered_error) < CENTERING_DEADBAND) {
    centre_prev_error = filtered_error;
    return 0;
  }
  
  centre_integral += filtered_error * dt;
  centre_integral = constrain(centre_integral, -200, 200);
  
  float derivative = (filtered_error - centre_prev_error) / dt;
  centre_prev_error = filtered_error;
  

  float correction = (centre_Kp * filtered_error) + 
                     (centre_Ki * centre_integral) +  
                     (centre_Kd * derivative);
  
  return constrain(correction, -50, 50);  
}

