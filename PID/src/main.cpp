#include <Arduino.h>
#include <Wire.h>

// Depth for heave
// IMU for yaw, stabilization (roll, pitch)
// 3rd sensor for sway, surge.

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

// ----------------------------------- Functions Prototypes -----------------------------------

// PID Control Functions 
float computePID(PID& myPID, float setPoint, float measuredValue);
void resetPID(PID& myPID);
void initPIDs(PID& myPID);
// Sensors Functions
float readDepth();
float readMPU();
float readXPos();
float readYPos();
void updateSensors();
// Control Functions
void setTargets(float depth, float yaw, float x, float y);
void setThrustersPower(Thruster_system& thruster, int Power); 
void applyThrusterSignal(Thruster_system& thruster);
void applyAllThrusterOutputs();
void setThrusterStatus(Thruster_system& mythruster, bool enabled);    // Enable or disable a thruster
void stopThruster(Thruster_system& mythruster);                       // Stops a thruster for a moment but keeps it enabled
void emergencyStop();                                                 // Stops ALL for a moment but keeps them enabled
void initThrusters();                                                 // Enables ALL and stops them

// ----------------------------------- Setup & loop -----------------------------------
void setup() {
  
}

void loop() {
  

}

// ----------------------------------- Functions Implementations -----------------------------------

// PID Control Functions
float computePID(PID& myPID, float setPoint, float measuredValue) {
  // Calculating Proportional Term
  unsigned long currentTime = millis();
  float dt = (currentTime - myPID.prevTime)/1000.0;
  if(dt<=0) return 0;
  float error = setPoint - measuredValue;
  // +ve error = below the target
  // -ve error = above it
  float P = myPID.kp * error;

  // Calculating Integral Term
  myPID.integral += error*dt;
  if(myPID.integral > myPID.integralLimit) myPID.integral = myPID.integralLimit;
  if(myPID.integral < -myPID.integralLimit) myPID.integral = -myPID.integralLimit;
  float I = myPID.ki*myPID.integral;
  
  // Calculating Derivative Term
  float de_dt = (error-myPID.prevError)/dt;
  float D = myPID.kd * de_dt;

  // Calculating The output
  float output = P + I + D;
  if(output > myPID.outputLimit) output = myPID.outputLimit;
  if(output < -myPID.outputLimit) output = -myPID.outputLimit;

  // Updating for next iteration
  myPID.prevError = error;
  myPID.prevTime = currentTime;
  
  return output;
}

void resetPID(PID& myPID) {
  myPID.prevError = 0;
  myPID.integral = 0;
  myPID.prevTime = millis();
  return;
}

void initPIDs() {
  unsigned long currentTime = millis();

  heavePID.prevTime = currentTime;
  yawPID.prevTime = currentTime;
  surgePID.prevTime = currentTime;
  swayPID.prevTime = currentTime;
}

// Sensors Functions
float readDepth() {

  return;
}

float readMPU() {

  return;
}

float readXPos() {
  // Computer Vision

  return;
}

float readYPos() {
  // Computer Vision

  return;
}

void updateSensors() {
  currentHeave = readDepth();
  currentYaw = readMPU();
  currentSurge = readXPos();
  currentSway = readYPos();

  return;
}

// Control Functions
void setTargets(float depth, float yaw, float x, float y) {
  targetHeave = depth;
  targetYaw = yaw;
  targetSurge = x;
  targetSway = y;

  return;
}

void setThrustersPower(Thruster_system& myThruster, int power) {
  if(!myThruster.isEnabled) {
    myThruster.power = 0;
    myThruster.pwmSignal = myThruster.neutralPwm;
    return;
  }

  power = constrain(power, -100,100);
  myThruster.power = power;

  if(!myThruster.power) 
    myThruster.pwmSignal = myThruster.neutralPwm;
  else
    myThruster.pwmSignal = map(myThruster.power,-100,100,myThruster.minPwm, myThruster.maxPwm);

  return;
}

void applyThrusterSignal(Thruster_system& myThruster) {
  int analogValue = map(myThruster.pwmSignal, 1000, 2000, 0, 255);
  analogWrite(myThruster.pin, analogValue);

  return;
}

void applyAllThrusterOutputs() {
  applyThrusterSignal(FR);
  applyThrusterSignal(FL);
  applyThrusterSignal(BR);
  applyThrusterSignal(BL);
  applyThrusterSignal(VL);
  applyThrusterSignal(VR);

  return;
}

void setThrusterStatus(Thruster_system& mythruster, bool enabled) {
  mythruster.isEnabled = enabled;
  if(!enabled) 
    stopThruster(mythruster);
  
  return;
}


void stopThruster(Thruster_system& mythruster) {
  mythruster.power = 0;
  mythruster.pwmSignal = mythruster.neutralPwm;
  applyThrusterSignal(mythruster);

  return;
}

void emergencyStop() {
  stopThruster(FR);
  stopThruster(FL);
  stopThruster(BR);
  stopThruster(BL);
  stopThruster(VR);
  stopThruster(VL);

  Serial.println("EMERGENCY STOP - All Thrusters Stopped!");

  return;
}

void initThrusters() {
  pinMode(FR.pin, OUTPUT);
  pinMode(FL.pin, OUTPUT);
  pinMode(BR.pin, OUTPUT);
  pinMode(BL.pin, OUTPUT);
  pinMode(VR.pin, OUTPUT);
  pinMode(VL.pin, OUTPUT);

  stopThruster(FR);
  stopThruster(FL);
  stopThruster(BR);
  stopThruster(BL);
  stopThruster(VR);
  stopThruster(VL);

  Serial.println("Thrusters initialized and stopped");

  return;
}

void  
