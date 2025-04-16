// Author: Ashutosh Singh
// Mar 21, 2025
// IBT-2 Motor Driver Control (200 RPM Motor)

const int LPWM1 = 5;
const int RPWM1 = 6;
const int L_EN1 = 7;
const int R_EN1 = 8;

const int LPWM2 = 9;
const int RPWM2 = 10;
const int L_EN2 = 11;
const int R_EN2 = 12;

const float wheelBase = 0.3; // Distance between left & right wheels (meters)
const int minPWM = 0;  // Minimum PWM to move motor (adjust if needed)
const int maxPWM = 255; // Maximum PWM (full power)
const float radius = 0.05; // Wheel radius in meters
const int maxRPM = 200; 

void setup() {
  Serial.begin(115200);

  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM1, OUTPUT);
  pinMode(L_EN1, OUTPUT);
  pinMode(R_EN1, OUTPUT);

  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(L_EN2, OUTPUT);
  pinMode(R_EN2, OUTPUT);

  // Enable motor drivers
  digitalWrite(L_EN1, HIGH);
  digitalWrite(R_EN1, HIGH);
  digitalWrite(L_EN2, HIGH);
  digitalWrite(R_EN2, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    float linear_vel = 0.0, angular_vel = 0.0;
    int separatorIndex = input.indexOf(',');
    if (separatorIndex != -1) {
      linear_vel = input.substring(0, separatorIndex).toFloat();
      angular_vel = input.substring(separatorIndex + 1).toFloat();
    }

    // Calculate wheel velocities (m/s)
    float leftWheelSpeed = (linear_vel/2) - ((angular_vel * wheelBase) / 2);
    float rightWheelSpeed = (linear_vel/2) + ((angular_vel * wheelBase) / 2);

    // Convert to RPM
    float RPM_L = abs((leftWheelSpeed / radius) * (60 / (2 * M_PI)));
    float RPM_R = abs((rightWheelSpeed / radius) * (60 / (2 * M_PI)));

    // Convert RPM to PWM (using 200 RPM max)
    int PWM_L = map(RPM_L, 0, maxRPM, minPWM, maxPWM);
    int PWM_R = map(RPM_R, 0, maxRPM, minPWM, maxPWM);

    // Constrain values to avoid overflow
    PWM_L = constrain(PWM_L, minPWM, maxPWM);
    PWM_R = constrain(PWM_R, minPWM, maxPWM);

    // Control Left Motor
    if (leftWheelSpeed > 0) {
      analogWrite(LPWM1, 0);
      analogWrite(RPWM1, PWM_L);
    } else if (leftWheelSpeed < 0) {
      analogWrite(LPWM1, PWM_L);
      analogWrite(RPWM1, 0);
    } else {
      analogWrite(LPWM1, 0);
      analogWrite(RPWM1, 0);
    }

    // Control Right Motor
    if (rightWheelSpeed > 0) {
      analogWrite(LPWM2, 0);
      analogWrite(RPWM2, PWM_R);
    } else if (rightWheelSpeed < 0) {
      analogWrite(LPWM2, PWM_R);
      analogWrite(RPWM2, 0);
    } else {
      analogWrite(LPWM2, 0);
      analogWrite(RPWM2, 0);
    }
  }
}
