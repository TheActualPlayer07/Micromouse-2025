#include "motor_control.h"
#include "turning_pid.h"
#include "centering_pid.h"
#include "forward_pid.h"

// Pin definitions
const int MOTOR_A_IN1 = 28;
const int MOTOR_A_IN2 = 29;
const int MOTOR_A_PWM = 3;
const int MOTOR_B_IN1 = 30;
const int MOTOR_B_IN2 = 31;
const int MOTOR_B_PWM = 5;

const int ENC_A1 = 17;
const int ENC_A2 = 16;
const int ENC_B1 = 15;
const int ENC_B2 = 14;

// Global variables
long encoderA_count = 0;
long encoderB_count = 0;
long prev_encoderA = 0;
long prev_encoderB = 0;
unsigned long prev_time = 0;
float speedA = 0;
float speedB = 0;

void initMotors() {
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  
  pinMode(ENC_A1, INPUT);
  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B1, INPUT);
  pinMode(ENC_B2, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, CHANGE);
  
  stopMotors();
}

void setMotors(float pwmA, float pwmB) {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_PWM, (int)pwmA);
  
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  analogWrite(MOTOR_B_PWM, (int)pwmB);
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_PWM, 0);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_B_PWM, 0);
}

void turnRight90(float target_yaw, float current_yaw) {
  stopMotors();
  delay(10);
  target_yaw = current_yaw + 90;
  turnToAngle(target_yaw, current_yaw);
  delay(10);
  resetPID();
  resetCenteringPID();
}

void turnLeft90(float target_yaw, float current_yaw) {
  stopMotors();
  delay(10);
  target_yaw = current_yaw - 90;
  turnToAngle(target_yaw, current_yaw);
  delay(10);
  resetPID();
  resetCenteringPID();
}

void calculateSpeeds() {
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;
  
  if (dt > 0.01) {
    long delta_A = encoderA_count - prev_encoderA;
    long delta_B = encoderB_count - prev_encoderB;
    
    speedA = delta_A / dt;
    speedB = delta_B / dt;
    
    prev_encoderA = encoderA_count;
    prev_encoderB = encoderB_count;
    prev_time = current_time;
  }
}

void resetEncoders() {
  encoderA_count = 0;
  encoderB_count = 0;
  prev_encoderA = 0;
  prev_encoderB = 0;
}

void encoderA_ISR() {
  encoderA_count++;
}

void encoderB_ISR() {
  encoderB_count++;
}
