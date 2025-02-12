// Author: Ashutosh Singh
// Jan 26, 2025

// Define motor control pins
const int leftMotorEnable = 3;  // Enable pin for left motor (PWM)
const int leftMotorIn1 = 4;     // IN1 pin for left motor
const int leftMotorIn2 = 7;     // IN2 pin for left motor
const int rightMotorEnable = 5; // Enable pin for right motor (PWM)
const int rightMotorIn1 = 6;    // IN1 pin for right motor
const int rightMotorIn2 = 8;    // IN2 pin for right motor

// Robot configuration
const float wheelBase = 0.3;    // Distance between wheels (meters)
//const float wheelRadius = 0.05; // Wheel radius (meters)
const int maxPWM = 255;         // Maximum PWM value

void setup() {
  Serial.begin(115200);

  pinMode(leftMotorEnable, OUTPUT);
  pinMode(leftMotorIn1, OUTPUT);
  pinMode(leftMotorIn2, OUTPUT);
  pinMode(rightMotorEnable, OUTPUT);
  pinMode(rightMotorIn1, OUTPUT);
  pinMode(rightMotorIn2, OUTPUT);

  // Stop motors initially
  analogWrite(leftMotorEnable, 0);
  analogWrite(rightMotorEnable, 0);
}

void loop() {
  // Check if data is available on the serial port
  if(Serial.available() > 0){
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove extra spaces and newlines
    
    float linear_vel = 0.0, angular_vel = 0.0;
    int separatorIndex = input.indexOf(',');
    if (separatorIndex != -1) {
      linear_vel = input.substring(0, separatorIndex).toFloat();
      angular_vel = input.substring(separatorIndex + 1).toFloat();
    }

    // Calculate wheel velocities
    float leftWheelSpeed = linear_vel - ((angular_vel * wheelBase) / 2);
    float rightWheelSpeed = linear_vel + ((angular_vel * wheelBase) / 2);

    // Convert speeds to PWM values
    int leftPWM = constrain(abs(leftWheelSpeed * 100), 0, maxPWM);
    int rightPWM = constrain(abs(rightWheelSpeed * 100), 0, maxPWM);
    
    if (leftWheelSpeed > 0) {
      digitalWrite(leftMotorIn1, HIGH);
      digitalWrite(leftMotorIn2, LOW);
    } else {
      digitalWrite(leftMotorIn1, LOW);
      digitalWrite(leftMotorIn2, HIGH);
    }

    if (rightWheelSpeed > 0) {
      digitalWrite(rightMotorIn1, HIGH);
      digitalWrite(rightMotorIn2, LOW);
    } else {
      digitalWrite(rightMotorIn1, LOW);
      digitalWrite(rightMotorIn2, HIGH);
    }
    analogWrite(leftMotorEnable, leftPWM);
    analogWrite(rightMotorEnable, rightPWM);
  }
}
