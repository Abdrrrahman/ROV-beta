#include <Arduino.h>
#include <Wire.h>


// PID Variables
struct PID {
  float kp, ki, kd;             
  float prevError;                      // To calculate Derivative
  unsigned long prevTime;               // To calculate Derivative
  float integral;                       // To calculate Integral
  float integralLimit, outputLimit;     // To put a limit for outputs
};

PID heavePID = {0,0,0,0,0,0,5,10};
PID yawPID = {0,0,0,0,0,0,5,10};
PID surgePID = {0,0,0,0,0,0,5,10};
PID swayPID = {0,0,0,0,0,0,5,10};

float currentHeave, currentYaw, currentSurge, currentSway;
float targetHeave, targetYaw, targetSurge, targetSway;


// Thrusters Variables
struct Thruster_system {
  int pin, power, pwmSignal, minPwm, maxPwm, neutralPwm;
  bool isEnabled;
  String name;
};

Thruster_system FR = {3,0,1500,1000,2000,1500, true,"Front_Right_Motor"};
Thruster_system FL = {5,0,1500,1000,2000,1500, true,"Front_Left_Motor"};

Thruster_system BR = {6,0,1500,1000,2000,1500, true,"Back_Right_Motor"};
Thruster_system BL = {9,0,1500,1000,2000,1500, true,"Back_Left_Motor"};

Thruster_system VR = {10,0,1500,1000,2000,1500, true,"Vertical_Right_Motor"};
Thruster_system VL = {11,0,1500,1000,2000,1500, true,"Vertical_Left_Motor"};


// Sensors Variables
const int DepthSensorPin = A0;

void setup() {
  

}

void loop() {
  


}
