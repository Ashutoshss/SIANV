// Author: Ashutosh Singh
// Mar 21, 2025
// IBT-2 Motor Driver Control

const int LPWM1 = 5;
const int RPWM1 = 6;
const int L_EN1 = 7;
const int R_EN1 = 8;

const int LPWM2 = 9;
const int RPWM2 = 10;
const int L_EN2 = 11;
const int R_EN2 = 12;

const float wheelBase = 0.3;
const int minPWM = 0;
const int maxPWM = 255;
const float radius = 0.05;

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

  digitalWrite(L_EN1, LOW);
  digitalWrite(R_EN1, LOW);
  digitalWrite(L_EN2, LOW);
  digitalWrite(R_EN2, LOW);
}

void loop() {
  digitalWrite(L_EN1, HIGH);
  digitalWrite(R_EN1, HIGH);
  digitalWrite(L_EN2, HIGH);
  digitalWrite(R_EN2, HIGH);
  
  if(Serial.available() > 0){
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    
    float linear_vel = 0.0, angular_vel = 0.0;
    int separatorIndex = input.indexOf(',');
    if (separatorIndex != -1) {
      linear_vel = input.substring(0, separatorIndex).toFloat();
      angular_vel = input.substring(separatorIndex + 1).toFloat();
    }

    // Calculate wheel velocities
    float leftWheelSpeed = linear_vel - ((angular_vel * wheelBase) / 2);
    float rightWheelSpeed = linear_vel + ((angular_vel * wheelBase) / 2);

    float RPM_L = abs((leftWheelSpeed / radius) * (60 / (2 * 3.14)));
    float RPM_R = abs((rightWheelSpeed / radius) * (60 / (2 * 3.14)));

    int PWM_L = map(RPM_L, 0, 150, minPWM, maxPWM); 
    int PWM_R = map(RPM_R, 0, 150, minPWM, maxPWM);

    PWM_L = constrain(PWM_L, minPWM, maxPWM);
    PWM_R = constrain(PWM_R, minPWM, maxPWM);

    if(leftWheelSpeed>0){
      analogWrite(LPWM1,RPM_L);
      analogWrite(RPWM1,0);
    }
    else if(leftWheelSpeed<0){
      analogWrite(LPWM1,0);
      analogWrite(RPWM1,RPM_L);
    }
    else{
      analogWrite(LPWM1,0);
      analogWrite(RPWM1,0);
    }

    if(rightWheelSpeed>0){
      analogWrite(LPWM2,RPM_R);
      analogWrite(RPWM2,0);
    }
    else if(rightWheelSpeed<0){
      analogWrite(LPWM2,0);
      analogWrite(RPWM2,RPM_R);
    }
    else{
      analogWrite(LPWM2,0);
      analogWrite(RPWM2,0);
    }
  }
}
