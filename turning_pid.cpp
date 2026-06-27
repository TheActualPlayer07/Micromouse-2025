#include "turning_pid.h"
#include "sensors.h"
#include "motor_control.h"

// PID gains
float turn_Kp = 2.5;
float turn_Ki = 0.01;
float turn_Kd = 0.3;

float turn_integral = 0;
float turn_prev_error = 0;

const float ANGLE_TOLERANCE = 0.25; 
const float MIN_TURN_PWM = 30;
const float MAX_TURN_PWM = 60;
const unsigned long TURN_TIMEOUT = 55000; 

void initTurningPID() {
  resetTurningPID();
}

void resetTurningPID() {
  turn_integral = 0;
  turn_prev_error = 0;
}

float normalizeAngle(float angle) {
  while (angle > 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}

float getAngleDifference(float target, float current) {
  float diff = target - current;
  if (diff > 180.0) diff -= 360.0;
  else if (diff < -180.0) diff += 360.0;
  return diff;
}

void turnToAngle(float target_angle, float current_angle) {
  resetTurningPID();
  target_angle = normalizeAngle(target_angle);
  
  unsigned long start_time = millis();
  unsigned long prev_pid_time = millis();
  
  Serial.print("Turning to ");
  Serial.println(target_angle);
  
  while (millis() - start_time < TURN_TIMEOUT) {
    unsigned long current_time = millis();
    float dt = (current_time - prev_pid_time) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prev_pid_time = current_time;

    float current_heading = getYaw();

    float error = getAngleDifference(target_angle, current_heading);
    
    if (abs(error) < ANGLE_TOLERANCE) {
      analogWrite(MOTOR_A_PWM, 0);
      analogWrite(MOTOR_B_PWM, 0);
      delay(50);  // Let momentum settle
      stopMotors();
      stopMotors();
      return;
    }
    turn_integral += error * dt;
    turn_integral = constrain(turn_integral, -30, 30);
    
    float derivative = (error - turn_prev_error) / dt;
    float output = turn_Kp * error + turn_Ki * turn_integral + turn_Kd * derivative;
    turn_prev_error = error;
    
    float pwm_val = constrain(abs(output), MIN_TURN_PWM, MAX_TURN_PWM);
    
    
    if (error < 0) { 
      // Turning "Positive" (Usually Left/CCW)
      // Left Motor BACK, Right Motor FWD
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
      
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
    } else {
      // Turning "Negative" (Usually Right/CW)
      // Left Motor FWD, Right Motor BACK
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
      
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
    }
    
    analogWrite(MOTOR_A_PWM, (int)pwm_val);
    analogWrite(MOTOR_B_PWM, (int)pwm_val);
    
    delay(20); // Keep loop stable
  }
  
  stopMotors(); // Timeout safety
}