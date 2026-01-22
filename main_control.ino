#include "sensors.h"
#include "motor_control.h"
#include "forward_pid.h"
#include "turning_pid.h"
#include "centering_pid.h"


const uint16_t WALL_THRESHOLD = 165;
const uint16_t FRONT_WALL_THRESHOLD = 100;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  initMotors();
  initSensors();
  initForwardPID();
  initTurningPID();
  initCenteringPID();
  Serial.println("\nStarting in 5 seconds...");
  Serial.println("Turn ON motor power!");
  delay(5000);
  resetEncoders();
}

void loop() {
  uint16_t front_dist = readFront();
  uint16_t left_dist = readLeft();
  uint16_t right_dist = readRight();
  
  bool wall_front = (front_dist < FRONT_WALL_THRESHOLD);
  bool wall_left = (left_dist < WALL_THRESHOLD);
  bool wall_right = (right_dist < WALL_THRESHOLD);

  if (wall_front) {
    stopMotors();
    delay(100);
    float current_yaw = getYaw();
    float target_yaw = 0;
    if (!wall_right && wall_left){
      Serial.println("\nTurning RIGHT");
      turnRight90(target_yaw, current_yaw);
      current_target_speed = 40.0;
    } else if (!wall_left && wall_right) {
      Serial.println("\nTurning LEFT");
      turnLeft90(target_yaw, current_yaw);
      current_target_speed = 40.0;
    } else if (!wall_left && !wall_right) {
      Serial.println("\nTurning LEFT (default)");
      turnLeft90(target_yaw, current_yaw);
      current_target_speed = 40.0;
    } else {
      Serial.println("\nDEAD END");
      turnLeft90(target_yaw, current_yaw);
      turnLeft90(target_yaw, current_yaw);
    }
  } else {

    calculateSpeeds();
    
    float desired_target = calculateTargetSpeed(front_dist);
    smoothRamp(desired_target);
    float centering_correction = 0;
    updatePID(current_target_speed, speedA, speedB);
    if(left_dist<175 && right_dist<175){
      centering_correction = computeCenteringCorrection(left_dist, right_dist);
    }
    else{
      centering_correction = 0;
    }
    float finalA = pwmA_out + centering_correction;
    float finalB = pwmB_out - centering_correction;
    finalA = constrain(finalA, 40, 200);
    finalB = constrain(finalB, 40, 200);
    setMotors(finalA, finalB);
    float current_yaw = getYaw();
    
    static unsigned long last_print = 0;
    if (millis() - last_print > 150) {
      Serial.print("Centering correction:");
      Serial.print(centering_correction);
      Serial.print("Front:");
      Serial.print(front_dist);
      Serial.print(" Left:");
      Serial.print(left_dist);
      Serial.print(" Right:");
      Serial.print(right_dist);
      Serial.print(" Yaw:");
      Serial.print(current_yaw);
      Serial.print(" | Tgt:");
      Serial.print(current_target_speed, 0);
      Serial.print(" SpdA:");
      Serial.print(speedA, 0);
      Serial.print(" SpdB:");
      Serial.print(speedB, 0);
      Serial.print(" | PWM:");
      Serial.print((int)pwmA_out);
      Serial.print("/");
      Serial.println((int)pwmB_out);
      last_print = millis();
    }
  }
  delay(20);
}
