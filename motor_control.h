#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// ADD THESE EXTERN DECLARATIONS FOR THE PINS
extern const int MOTOR_A_IN1;
extern const int MOTOR_A_IN2;
extern const int MOTOR_A_PWM;
extern const int MOTOR_B_IN1;
extern const int MOTOR_B_IN2;
extern const int MOTOR_B_PWM;

// Encoder counts
extern long encoderA_count;
extern long encoderB_count;

// Speed values
extern float speedA;
extern float speedB;

// Functions
void initMotors();
void setMotors(float pwmA, float pwmB);
void stopMotors();
void calculateSpeeds();
void resetEncoders();
void turnRight90();
void turnLeft90();

// ISR functions
void encoderA_ISR();
void encoderB_ISR();

#endif
