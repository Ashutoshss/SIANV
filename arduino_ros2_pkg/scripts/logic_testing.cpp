// this script is to test the pwm calculation which is been implemented over the hardware and to debug the problems

#include <iostream>
#include <cmath>

const float wheelBase = 0.3;
const float radius = 0.05;
const int minPWM = 0;  
const int maxPWM = 255;
const int maxRPM = 200;

int mapValue(float x, float in_min, float in_max, int out_min, int out_max) {
    return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int constrain(int x, int a, int b) {
    return std::max(a, std::min(x, b));
}

void Calculate(float linear_vel, float angular_vel) {
    float leftWheelSpeed = (linear_vel/2) - ((angular_vel * wheelBase) / 2);
    float rightWheelSpeed = (linear_vel/2) + ((angular_vel * wheelBase) / 2);

    // Convert to RPM
    float RPM_L = std::abs((leftWheelSpeed / radius) * (60 / (2 * M_PI)));
    float RPM_R = std::abs((rightWheelSpeed / radius) * (60 / (2 * M_PI)));

    // Convert RPM to PWM
    int PWM_L = mapValue(RPM_L, 0, maxRPM, minPWM, maxPWM);
    int PWM_R = mapValue(RPM_R, 0, maxRPM, minPWM, maxPWM);

    PWM_L = constrain(PWM_L, minPWM, maxPWM);
    PWM_R = constrain(PWM_R, minPWM, maxPWM);

    std::cout << "msg.linear.x = " << linear_vel << ", Left Wheel: Speed = " << leftWheelSpeed << " m/s, RPM = " << RPM_L << ", PWM = " << PWM_L << std::endl;
    std::cout << "msg.angular.z = " << angular_vel << ", Right Wheel: Speed = " << rightWheelSpeed << " m/s, RPM = " << RPM_R << ", PWM = " << PWM_R << std::endl;
}

int main() {
    Calculate(1.0, 0.0);
    Calculate(0.0, 1.0);
    Calculate(1.0, 1.0);
    return 0;
}

