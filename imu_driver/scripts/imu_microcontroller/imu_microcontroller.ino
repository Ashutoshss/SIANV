#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>  

MPU6050 imu;
Madgwick filter;  

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.initialize();

  if (imu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  imu.getAcceleration(&ax, &ay, &az);
  imu.getRotation(&gx, &gy, &gz);

  // Convert raw accelerometer values in m/s^2 (assuming ±2g range)
  float accelX = ax / 16384.0 * 9.81;
  float accelY = ay / 16384.0 * 9.81;
  float accelZ = az / 16384.0 * 9.81;

  // Convert raw gyroscope values to rad/s (assuming ±250°/s range)
  float gyroX = gx / 131.0 * (M_PI / 180.0);
  float gyroY = gy / 131.0 * (M_PI / 180.0);
  float gyroZ = gz / 131.0 * (M_PI / 180.0);

  filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

  // quaternion orientation
  float q_w = filter.qw;
  float q_x = filter.qx;
  float q_y = filter.qy;
  float q_z = filter.qz;

  Serial.print(q_x); Serial.print(", ");
  Serial.print(q_y); Serial.print(", ");
  Serial.print(q_z); Serial.print(", ");
  Serial.print(q_w); Serial.print(", ");

  Serial.print(accelX); Serial.print(", ");
  Serial.print(accelY); Serial.print(", ");
  Serial.print(accelZ); Serial.print(", ");

  Serial.print(gyroX); Serial.print(", ");
  Serial.print(gyroY); Serial.print(", ");
  Serial.print(gyroZ); Serial.print("\n");

  delay(10);
}