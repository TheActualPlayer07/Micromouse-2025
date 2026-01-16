
// #include <Wire.h>
// #include <VL53L0X.h>

// // Motors

// const int MOTOR_A_IN1 = 28;
// const int MOTOR_A_IN2 = 29;
// const int MOTOR_A_PWM = 3;
// const int MOTOR_B_IN1 = 30;
// const int MOTOR_B_IN2 = 31;
// const int MOTOR_B_PWM = 5;

// // Encoders
// const int ENC_A1 = 17;
// const int ENC_A2 = 16;
// const int ENC_B1 = 15;
// const int ENC_B2 = 14;

// // Sensor XSHUT pins
// const int XSHUT_1 = 24;
// const int XSHUT_2 = 25;
// const int XSHUT_3 = 26;

// VL53L0X frontSensor;

// long encoderA_count = 0;
// long encoderB_count = 0;
// long prev_encoderA = 0;
// long prev_encoderB = 0;
// unsigned long prev_time = 0;
// float speedA = 0;
// float speedB = 0;

// float speed_Kp = 1.0;
// float speed_Ki = 0.05;
// float speed_Kd = 0.05;
// float speed_integralA = 0;
// float speed_integralB = 0;
// float speed_prev_errorA = 0;
// float speed_prev_errorB = 0;

// float base_speed = 50.0;
// float current_target = 0.0;

// const float RAMP_UP_PER_LOOP = 1.0;     // target speed increase per loop (every 20 ms)
// const float BRAKE_DOWN_PER_LOOP = 3.0;  // target speed decrease per loop (brake faster)


// // ===== SETUP =====
// void setup() {
//   Serial.begin(115200);
//   while (!Serial && millis() < 3000);
  
//   Serial.println("Forward PID - SMOOTH Motion");
  
//   pinMode(MOTOR_A_IN1, OUTPUT);
//   pinMode(MOTOR_A_IN2, OUTPUT);
//   pinMode(MOTOR_A_PWM, OUTPUT);
//   pinMode(MOTOR_B_IN1, OUTPUT);
//   pinMode(MOTOR_B_IN2, OUTPUT);
//   pinMode(MOTOR_B_PWM, OUTPUT);
  
//   pinMode(ENC_A1, INPUT);
//   pinMode(ENC_A2, INPUT);
//   pinMode(ENC_B1, INPUT);
//   pinMode(ENC_B2, INPUT);
//   attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, CHANGE);
  
//   pinMode(XSHUT_1, OUTPUT);
//   pinMode(XSHUT_2, OUTPUT);
//   pinMode(XSHUT_3, OUTPUT);
  
//   digitalWrite(XSHUT_1, LOW);
//   digitalWrite(XSHUT_2, LOW);
//   digitalWrite(XSHUT_3, LOW);
//   delay(50);
  
//   Wire.begin();
  
//   Serial.println("Initializing front sensor");
//   digitalWrite(XSHUT_2, HIGH);
//   delay(50);
  
//   frontSensor.setTimeout(500);
//   if (!frontSensor.init()) {
//     Serial.println("ERROR: Front sensor failed!");
//     while(1);
//   }
//   Serial.println("Front sensor OK");
  
//   stopMotors();
  
//   Serial.println("\nStarting in 7 seconds...");
//   Serial.println("Place wall 100-200cm ahead");
//   Serial.println("Turn ON motor power switch if off(move the black thing on the left)!\n");
//   delay(7000);
  
//   resetPID();
//   encoderA_count = 0;
//   encoderB_count = 0;
//   current_target = 30.0;
//   prev_time = millis();
// }

// void loop() {
//   uint16_t front_distance = frontSensor.readRangeSingleMillimeters();
//   if (frontSensor.timeoutOccurred()) front_distance = 2000;
  
//   if (front_distance < 100) {
//     Serial.println("\nWALL DETECTED - STOPPING");
//     stopMotors();
//     Serial.print("Final distance to wall: ");
//     Serial.print(front_distance);
//     Serial.println("mm");

//     while(1);
//   }
  
//   calculateSpeeds();
  
//   float calculated_target = calculateTargetSpeed(front_distance);

//   // Smooth accel + braking by limiting how fast the target can change
//   if (calculated_target > current_target) {
//     current_target = min(current_target + RAMP_UP_PER_LOOP, calculated_target);
//   } else {
//     current_target = max(current_target - BRAKE_DOWN_PER_LOOP, calculated_target);
//   }

  
//   float speed_errorA = current_target - speedA;
//   float speed_errorB = current_target - speedB;
  
//   speed_integralA += speed_errorA * 0.02;
//   speed_integralB += speed_errorB * 0.02;
//   speed_integralA = constrain(speed_integralA, -50, 50); 
//   speed_integralB = constrain(speed_integralB, -50, 50);
  
//   float speed_derivativeA = (speed_errorA - speed_prev_errorA) / 0.02;
//   float speed_derivativeB = (speed_errorB - speed_prev_errorB) / 0.02;
  
//   float pwmA = speed_Kp * speed_errorA + speed_Ki * speed_integralA + speed_Kd * speed_derivativeA;
//   float pwmB = speed_Kp * speed_errorB + speed_Ki * speed_integralB + speed_Kd * speed_derivativeB;
  
//   speed_prev_errorA = speed_errorA;
//   speed_prev_errorB = speed_errorB;
  
//   pwmA = constrain(pwmA, 40, 200);
//   pwmB = constrain(pwmB, 40, 200);
  
//   setMotors(pwmA, pwmB);
  
//   static unsigned long last_print = 0;
//   if (millis() - last_print > 100) {
//     Serial.print("Dist: ");
//     Serial.print(front_distance);
//     Serial.print("mm  DesiredTarget: ");
//     Serial.print(calculated_target, 0);
//     Serial.print("  CurrentTarget: ");
//     Serial.print(current_target, 0);
//     Serial.print("  SpeedA: ");
//     Serial.print(speedA, 0);
//     Serial.print("  SpeedB: ");
//     Serial.print(speedB, 0);
//     Serial.print("  PWM: ");
//     Serial.print((int)pwmA);
//     Serial.print("/");
//     Serial.println((int)pwmB);
//     last_print = millis();
//   }
  
//   delay(20);
// }

// float calculateTargetSpeed(uint16_t distance) {
//   if (distance > 500) {
//     return base_speed;
//   } else if (distance > 300) {
//     // return base_speed * 0.6;
//     return map(distance, 300, 500, base_speed * 0.6, base_speed);
//   } else if (distance > 150) {
//     return map(distance, 150, 300, base_speed * 0.3, base_speed * 0.6);
//   } else if (distance > 50) {
//     return base_speed * 0.1;
//   } else {
//     return base_speed * 0.05;
//   }
// }

// void calculateSpeeds() {
//   unsigned long current_time = millis();
//   float dt = (current_time - prev_time) / 1000.0;
  
//   if (dt > 0.01) {
//     long delta_A = encoderA_count - prev_encoderA;
//     long delta_B = encoderB_count - prev_encoderB;
    
//     speedA = delta_A / dt;
//     speedB = delta_B / dt;
    
//     prev_encoderA = encoderA_count;
//     prev_encoderB = encoderB_count;
//     prev_time = current_time;
//   }
// }

// void setMotors(float pwmA, float pwmB) {
//   digitalWrite(MOTOR_A_IN1, HIGH);
//   digitalWrite(MOTOR_A_IN2, LOW);
//   analogWrite(MOTOR_A_PWM, (int)pwmA);
  
//   digitalWrite(MOTOR_B_IN1, LOW);
//   digitalWrite(MOTOR_B_IN2, HIGH);
//   analogWrite(MOTOR_B_PWM, (int)pwmB);
// }

// void stopMotors() {
//   digitalWrite(MOTOR_A_IN1, LOW);
//   digitalWrite(MOTOR_A_IN2, LOW);
//   analogWrite(MOTOR_A_PWM, 0);
//   digitalWrite(MOTOR_B_IN1, LOW);
//   digitalWrite(MOTOR_B_IN2, LOW);
//   analogWrite(MOTOR_B_PWM, 0);
// }

// void resetPID() {
//   speed_integralA = 0;
//   speed_integralB = 0;
//   speed_prev_errorA = 0;
//   speed_prev_errorB = 0;
// }

// void encoderA_ISR() {
//   encoderA_count++;
// }

// void encoderB_ISR() {
//   encoderB_count++;
// }
