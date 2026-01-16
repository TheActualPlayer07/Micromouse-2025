#include "forward_pid.h"

// PID gains
float speed_Kp = 1.0;
float speed_Ki = 0.05;
float speed_Kd = 0.05;
float speed_integralA = 0;
float speed_integralB = 0;
float speed_prev_errorA = 0;
float speed_prev_errorB = 0;


unsigned long prev_pid_time = 0;


// Speed settings
float base_speed = 150.0;
float current_target_speed = 0.0;

// Output PWM
float pwmA_out = 0;
float pwmB_out = 0;

// Ramp rates
const float RAMP_UP_PER_LOOP = 2.0;
const float BRAKE_DOWN_PER_LOOP = 5.0;

void initPID() {
  resetPID();
  current_target_speed = 0.0;
  prev_pid_time = millis();
}

void resetPID() {
  speed_integralA = 0;
  speed_integralB = 0;
  speed_prev_errorA = 0;
  speed_prev_errorB = 0;
  prev_pid_time = millis();
}

void smoothRamp(float desired_target) {
  if (desired_target > current_target_speed) {
    current_target_speed = min(current_target_speed + RAMP_UP_PER_LOOP, desired_target);
  } else {
    current_target_speed = max(current_target_speed - BRAKE_DOWN_PER_LOOP, desired_target);
  }
}

void updatePID(float targetSpeed, float currentSpeedA, float currentSpeedB) {
  unsigned long current_time = millis();
  float dt = (current_time - prev_pid_time) / 1000.0;

  if(dt <= 0.0) dt = 0.001; 
  prev_pid_time = current_time;

  float speed_errorA = targetSpeed - currentSpeedA;
  float speed_errorB = targetSpeed - currentSpeedB;
  
  speed_integralA += speed_errorA * dt;
  speed_integralB += speed_errorB * dt;
  speed_integralA = constrain(speed_integralA, -50, 50);
  speed_integralB = constrain(speed_integralB, -50, 50);
  
  float speed_derivativeA = (speed_errorA - speed_prev_errorA) / dt;
  float speed_derivativeB = (speed_errorB - speed_prev_errorB) / dt;
  
  pwmA_out = speed_Kp * speed_errorA + speed_Ki * speed_integralA + speed_Kd * speed_derivativeA;
  pwmB_out = speed_Kp * speed_errorB + speed_Ki * speed_integralB + speed_Kd * speed_derivativeB;
  
  speed_prev_errorA = speed_errorA;
  speed_prev_errorB = speed_errorB;
  
  pwmA_out = constrain(pwmA_out, 40, 200);
  pwmB_out = constrain(pwmB_out, 40, 200);
}

float calculateTargetSpeed(uint16_t distance) {
  if (distance > 500) {
    return base_speed;
  } else if (distance > 300) {
    return map(distance, 300, 500, base_speed * 0.6, base_speed);
  } else if (distance > 150) {
    return map(distance, 150, 300, base_speed * 0.3, base_speed * 0.6);
  } else if (distance > 50) {
    return base_speed * 0.1;
  } else {
    return 0;
  }
}
