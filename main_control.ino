#include "sensors.h"
#include "motor_control.h"
#include "forward_pid.h"
#include "turning_pid.h"

const uint16_t WALL_THRESHOLD = 165;
const uint16_t FRONT_WALL_THRESHOLD = 100;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println("Wall Following Robot");
  
  initMotors();
  initSensors();
  initPID();
  initTurningPID();
  
  
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
  
  // Decision logic
  if (wall_front) {
    stopMotors();
    delay(100);
    
    float current_yaw = getYaw();
    float target_yaw = 0;
    if (!wall_right && wall_left) {
      // Space on right -> turn right
      Serial.println("\nTurning RIGHT");
      stopMotors();
      delay(10);
      // turnRight90();
      target_yaw = current_yaw + 90;
      turnToAngle(target_yaw, current_yaw);
      delay(10);
      resetPID();
      current_target_speed = 40.0;
    } else if (!wall_left && wall_right) {
      // Space on left -> turn left
      Serial.println("\nTurning LEFT");
      stopMotors();
      delay(10);
      // turnLeft90();
      target_yaw = current_yaw - 90;
      turnToAngle(target_yaw, current_yaw);
      delay(10);
      resetPID();
      current_target_speed = 40.0;
    } else if (!wall_left && !wall_right) {
      // Both sides open -> default left
      Serial.println("\nTurning LEFT (default)");
      stopMotors();
      delay(10);
      // turnLeft90();
      target_yaw = current_yaw - 90;
      turnToAngle(target_yaw, current_yaw);
      delay(10);
      resetPID();
      current_target_speed = 40.0;
    } else {
      // Dead end
      Serial.println("\nDEAD END");
      stopMotors();
      target_yaw = current_yaw + 180;
      turnToAngle(target_yaw, current_yaw);
    }
  } else {
    // Continue forward
    calculateSpeeds();
    
    float desired_target = calculateTargetSpeed(front_dist);
    smoothRamp(desired_target);
    
    updatePID(current_target_speed, speedA, speedB);
    setMotors(pwmA_out, pwmB_out);
    float current_yaw = getYaw();
    
    // Debug print
    static unsigned long last_print = 0;
    if (millis() - last_print > 150) {
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
