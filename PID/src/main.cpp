#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// Depth for heave
// IMU for yaw, stabilization (roll, pitch)
// 3rd sensor for sway, surge.

Servo FR_ESC;
Servo FL_ESC;
Servo BR_ESC;
Servo BL_ESC;
Servo VR_ESC;
Servo VL_ESC; 

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
void resetALL_PIDs();
void resetPID(PID& myPID);
void initPIDs();
// Sensors Functions
float readDepth();
float readMPU();
float readXPos();
float readYPos();
void updateSensors();
float normalizeAngle(float angle);
// Control Functions
void setThrustersPower(Thruster_system& thruster, int Power); 
void applyThrusterSignal(Thruster_system& thruster, Servo& myServo);
void applyAllThrusterOutputs();
void setThrusterStatus(Thruster_system& mythruster,Servo& myServo, bool enabled);   // Enables or disables a thruster
void setALLThrustersStatus(bool enabled);                                           // Enables or disables ALL thrusters
void setVerticalThrustersStatus(bool enabled);                                      // Enables or disables Vertical thrusters
void setHorizontalThrustersStatus(bool enabled);                                    // Enables or disables Horizontal thrusters
void stopThruster(Thruster_system& mythruster, Servo& myServo);                     // Stops a thruster for a moment but keeps it enabled
void emergencyStop();                                                               // Stops ALL for a moment but keeps them enabled
void stopHorizontal();
void stopVertical();
void initThrusters();                                                               // Enables ALL and stops them
void mixThrusters(float heave, float yaw, float surge, float sway);
// High Control Functions
void setTargets(float depth, float yaw, float surge, float sway);
void computeControl();
// Debug Functions
void printDOFInfo();
void printThrusterInfo(Thruster_system& myThruster);
void printHorizontalThrustersInfo();
void printVerticalThrustersInfo();
void printALLThrustersPower();
void printALLThrustersInfo();
void testThruster(String name, int power);
void checkForSerialCommand();


// ----------------------------------- Setup & loop -----------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  FR_ESC.attach(FR.pin);
  FL_ESC.attach(FL.pin);
  BR_ESC.attach(BR.pin);
  BL_ESC.attach(BL.pin);
  VR_ESC.attach(VR.pin);
  VL_ESC.attach(VL.pin);

  initThrusters();
  initPIDs();

  Serial.println("ROV System initialized.");
  Serial.println("Type \"help\" to show commands.");
}

void loop() {
  setTargets(0.0,0.0,0.0,0.0);
  computeControl();
  applyAllThrusterOutputs();
  checkForSerialCommand();

  static unsigned long lastDebug = 0;   // Not set to zero every iteration due to static keyword
  if(millis() - lastDebug > 1000) {
    printALLThrustersPower();
    lastDebug = millis();
  }

  delay(5);
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

void resetALL_PIDs() {
  resetPID(heavePID);
  resetPID(yawPID);
  resetPID(surgePID);
  resetPID(swayPID);

  Serial.println("All PIDs reset");
  return;
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

  return 0.0;
}

float readMPU() {

  return 0.0;
}

float readXPos() {
  // Computer Vision

  return 0.0;
}

float readYPos() {
  // Computer Vision

  return 0.0;
}

void updateSensors() {
  currentHeave = readDepth();
  currentYaw = readMPU();
  currentSurge = readXPos();
  currentSway = readYPos();

  return;
}

float normalizeAngle(float angle) {
  while(angle<-180) angle+=360;
  while(angle>180) angle-=360;
  
  return angle;
}

// Control Functions
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

void applyThrusterSignal(Thruster_system& myThruster, Servo& myServo) {
  myServo.writeMicroseconds(myThruster.pwmSignal);

  return;
}

void applyAllThrusterOutputs() {
  applyThrusterSignal(FR, FR_ESC);
  applyThrusterSignal(FL, FL_ESC);
  applyThrusterSignal(BR, BR_ESC);
  applyThrusterSignal(BL, BL_ESC);
  applyThrusterSignal(VL, VL_ESC);
  applyThrusterSignal(VR, VR_ESC);

  return;
}

void setThrusterStatus(Thruster_system& mythruster,Servo& myServo, bool enabled) {
  mythruster.isEnabled = enabled;
  if(!enabled) 
    stopThruster(mythruster, myServo);
  
  return;
}

void setALLThrustersStatus(bool enabled) {
  FR.isEnabled = enabled;
  FL.isEnabled = enabled;
  BR.isEnabled = enabled;
  BL.isEnabled = enabled;
  VR.isEnabled = enabled;
  VL.isEnabled = enabled;
  if(!enabled) {
    emergencyStop();
  }
  
  return;
}

void setVerticalThrustersStatus(bool enabled) {
  VR.isEnabled = enabled;
  VL.isEnabled = enabled;
  if(!enabled) {
    stopThruster(VR, VR_ESC);
    stopThruster(VL, VL_ESC); 
  }
  
  return;
}

void setHorizontalThrustersStatus(bool enabled) {
  FR.isEnabled = enabled;
  FL.isEnabled = enabled;
  BR.isEnabled = enabled;
  BL.isEnabled = enabled;
  if(!enabled) {
    stopThruster(FR, FR_ESC);
    stopThruster(FL, FL_ESC);
    stopThruster(BR, BR_ESC);
    stopThruster(BL, BL_ESC);
  }
  
  return;
}


void stopThruster(Thruster_system& mythruster, Servo& myServo) {
  mythruster.power = 0;
  mythruster.pwmSignal = mythruster.neutralPwm;
  applyThrusterSignal(mythruster, myServo );

  return;
}

void emergencyStop() {
  stopThruster(FR, FR_ESC);
  stopThruster(FL, FL_ESC);
  stopThruster(BR, BR_ESC);
  stopThruster(BL, BL_ESC);
  stopThruster(VR, VR_ESC);
  stopThruster(VL, VL_ESC);

  Serial.println("EMERGENCY STOP - All Thrusters Stopped!");

  return;
}
void stopHorizontal() {
  stopThruster(FR, FR_ESC);
  stopThruster(FL, FL_ESC);
  stopThruster(BR, BR_ESC);
  stopThruster(BL, BL_ESC);

  Serial.println("HORIZONTAL STOP!");

  return;
}
void stopVertical() {
  stopThruster(VR, VR_ESC);
  stopThruster(VL, VL_ESC);

  Serial.println("VERICAL STOP");

  return;
}

void initThrusters() {
  pinMode(FR.pin, OUTPUT);
  pinMode(FL.pin, OUTPUT);
  pinMode(BR.pin, OUTPUT);
  pinMode(BL.pin, OUTPUT);
  pinMode(VR.pin, OUTPUT);
  pinMode(VL.pin, OUTPUT);

  stopThruster(FR, FR_ESC);
  stopThruster(FL, FL_ESC);
  stopThruster(BR, BR_ESC);
  stopThruster(BL, BL_ESC);
  stopThruster(VR, VR_ESC);
  stopThruster(VL, VL_ESC);

  Serial.println("Thrusters initialized and stopped.");

  return;
}

void mixThrusters(float heave, float yaw, float surge, float sway) {
  // Constants
  const float INV_SQRT_2 = 0.7071067812; // = cos(45) = sin(45)

  // Verticals 
  setThrustersPower(VL, heave);
  setThrustersPower(VR, heave);

  // Horizontals
  
  // NorthEast 45 degrees (Front Right)
  float FR_surge_component = surge * INV_SQRT_2;
  float FR_sway_component = sway * INV_SQRT_2;
  float FR_yaw_component = yaw;
  setThrustersPower(FR,FR_surge_component+FR_sway_component+FR_yaw_component);

  // NortWest 135 degrees (Front Left)
  float FL_surge_component = surge * INV_SQRT_2;
  float FL_sway_component = -sway * INV_SQRT_2;
  float FL_yaw_component = -yaw;
  setThrustersPower(FL,FL_surge_component + FL_sway_component + FL_yaw_component);
  
  // SouthWest 225 degrees (Back Left)
  float BL_surge_component = -surge * INV_SQRT_2;
  float BL_sway_component = sway * INV_SQRT_2;
  float BL_yaw_component = -yaw;
  setThrustersPower(BL,BL_surge_component + BL_sway_component + BL_yaw_component);
  
  // SouthEast 315 degrees (Back Right)
  float BR_surge_component = -surge * INV_SQRT_2;
  float BR_sway_component = -sway * INV_SQRT_2;
  float BR_yaw_component = +yaw;
  setThrustersPower(BR,BR_surge_component + BR_sway_component + BR_yaw_component);
 
  return;
}

// High Level Control
void setTargets(float depth, float yaw, float surge, float sway) {
  targetHeave = depth;
  targetYaw = yaw;
  targetSurge = surge;
  targetSway = sway;

  return;
}

void computeControl() {
  updateSensors();

  float heave = computePID(heavePID, targetHeave, currentHeave);
  float yaw = computePID(yawPID, targetYaw, normalizeAngle(currentYaw));
  float surge = computePID(surgePID, targetSurge, currentSurge);
  float sway = computePID(swayPID, targetSway, currentSway);

  mixThrusters(heave, yaw, surge, sway);

  return;
}

void printDOFInfo() {
  Serial.println();

  Serial.print("Heave: "); Serial.print(currentHeave);
  Serial.print(", Target: "); Serial.print(targetHeave); Serial.println(".");

  Serial.print("Yaw: "); Serial.print(currentYaw);
  Serial.print(", Target: "); Serial.print(targetYaw); Serial.println(".");

  Serial.print("Surge: "); Serial.print(currentSurge);
  Serial.print(", Target: "); Serial.print(targetSurge); Serial.println(".");

  Serial.print("Sway: "); Serial.print(currentSway);
  Serial.print(", Target: "); Serial.print(targetSway); Serial.println(".");

  Serial.println();

  return;
}

void printThrusterInfo(Thruster_system& myThruster) {
  Serial.println("-> ");

  Serial.print(myThruster.name); Serial.println(" info:");
  Serial.print("Pin: "); Serial.print(myThruster.pin); Serial.println(".");
  Serial.print("Power: "); Serial.print(myThruster.power); Serial.println(".");
  Serial.print("PWM: "); Serial.print(myThruster.pwmSignal); Serial.println(".");
  Serial.print("Status: "); Serial.println(myThruster.isEnabled? "Enabled.":"Disabled.");

  return;
}

void printHorizontalThrustersInfo() {
  
  Serial.println();

  printThrusterInfo(FR);
  printThrusterInfo(FL);
  printThrusterInfo(BR);
  printThrusterInfo(BL);
  
  Serial.println();

  return;
}

void printVerticalThrustersInfo() {
  
  Serial.println();

  printThrusterInfo(VR);
  printThrusterInfo(VL);
  
  Serial.println();

  return;
}
void printALLThrustersPower() {
  Serial.print("Thrusters: ");
  Serial.print("FR: "); Serial.println(FR.power);
  Serial.print("FL: "); Serial.println(FL.power);
  Serial.print("BR: "); Serial.println(BR.power);
  Serial.print("BL: "); Serial.println(BL.power);
  Serial.print("VR: "); Serial.println(VR.power);
  Serial.print("VL: "); Serial.println(VL.power);
  Serial.println();
}

void printALLThrustersInfo() {
  
  Serial.println();

  printThrusterInfo(FR);
  printThrusterInfo(FL);
  printThrusterInfo(BR);
  printThrusterInfo(BL);
  printThrusterInfo(VR);
  printThrusterInfo(VL);
  
  Serial.println();

  return;
}

void testThruster(String name, int power) {

  emergencyStop();
  Serial.println();
  if(name.equalsIgnoreCase("FR")) {
    setThrustersPower(FR, power);
    applyThrusterSignal(FR, FR_ESC);
    Serial.print("Testing "); Serial.print(FR.name);
    Serial.print(" at power "); Serial.print(FR.power); Serial.println(".");
  } 
  else if(name.equalsIgnoreCase("FL")) {
      setThrustersPower(FL, power);
      applyThrusterSignal(FL, FL_ESC);
      Serial.print("Testing "); Serial.print(FL.name);
      Serial.print(" at power "); Serial.print(FL.power); Serial.println(".");
  } 
  else if(name.equalsIgnoreCase("BL")) {
      setThrustersPower(BL, power);
      applyThrusterSignal(BL, BL_ESC);
      Serial.print("Testing "); Serial.print(BL.name);
      Serial.print(" at power "); Serial.print(BL.power); Serial.println(".");
  } 
  else if(name.equalsIgnoreCase("BR")) {
      setThrustersPower(BR, power);
      applyThrusterSignal(BR, BR_ESC);
      Serial.print("Testing "); Serial.print(BR.name);
      Serial.print(" at power "); Serial.print(BR.power); Serial.println(".");
  } 
  else if(name.equalsIgnoreCase("VR")) {
      setThrustersPower(VR, power);
      applyThrusterSignal(VR, VR_ESC);
      Serial.print("Testing "); Serial.print(VR.name);
      Serial.print(" at power "); Serial.print(VR.power); Serial.println(".");
  } 
  else if(name.equalsIgnoreCase("VL")) {
      setThrustersPower(VL, power);
      applyThrusterSignal(VL, VL_ESC);
      Serial.print("Testing "); Serial.print(VL.name);
      Serial.print(" at power "); Serial.print(VL.power); Serial.println(".");
  } 
  else {
    Serial.print("Thruster "); Serial.print(name); Serial.println(" Not Found!");
  }

  Serial.println();

  return;
}

void checkForSerialCommand() {
  if(Serial.available()) {
    String command = Serial.readString();
    command.trim();

    // PID tuning commands  
    if(command.startsWith("heave -kp ")) {
      heavePID.kp = command.substring(10).toFloat();
      Serial.print("Heave Kp updated to "); Serial.print(heavePID.kp); Serial.println(".");
    }
    else if (command.startsWith("heave -ki ")) {
      heavePID.ki = command.substring(10).toFloat();
      Serial.print("Heave Ki updated to "); Serial.print(heavePID.ki); Serial.println(".");
    }
    else if (command.startsWith("heave -kd ")) {
      heavePID.kd = command.substring(10).toFloat();
      Serial.print("Heave Kd updated to "); Serial.print(heavePID.kd); Serial.println(".");
    }
    else if (command.startsWith("heave -target ")) {
      targetHeave = command.substring(14).toFloat();
      Serial.print("Target Heave updated to "); Serial.print(targetHeave); Serial.println(".");
    }
    else if (command.startsWith("yaw -kp ")) {
      yawPID.kp = command.substring(8).toFloat();
      Serial.print("Yaw Kp updated to "); Serial.print(yawPID.kp); Serial.println(".");
    }
    else if (command.startsWith("yaw -ki ")) {
      yawPID.ki = command.substring(8).toFloat();
      Serial.print("Yaw Ki updated to "); Serial.print(yawPID.ki); Serial.println(".");
    }
    else if (command.startsWith("yaw -kd ")) {
      yawPID.kd = command.substring(8).toFloat();
      Serial.print("Yaw Kd updated to "); Serial.print(yawPID.kd); Serial.println(".");
    }
    else if (command.startsWith("yaw -target ")) {
      targetYaw = command.substring(12).toFloat();
      Serial.print("Target Yaw updated to "); Serial.print(targetYaw); Serial.println(".");
    }
    else if (command.startsWith("surge -kp ")) {
      surgePID.kp = command.substring(10).toFloat();
      Serial.print("Surge Kp is updated to "); Serial.print(surgePID.kp); Serial.println(".");
    }
    else if (command.startsWith("surge -ki ")) {
      surgePID.ki = command.substring(10).toFloat();
      Serial.print("Surge Ki updated to "); Serial.print(surgePID.ki); Serial.println(".");
    }
    else if (command.startsWith("surge -kd ")) {
      surgePID.kd = command.substring(10).toFloat();
      Serial.print("Surge Kd updated to "); Serial.print(surgePID.kd); Serial.println(".");
    }
    else if (command.startsWith("surge -target ")) {
      targetSurge = command.substring(14).toFloat();
      Serial.print("Target surge updated to "); Serial.print(targetSurge); Serial.println(".");
    }
    else if (command.startsWith("sway -kp ")) {
      swayPID.kp = command.substring(9).toFloat();
      Serial.print("Sway Kp updated to "); Serial.print(swayPID.kp); Serial.println(".");
    }
    else if (command.startsWith("sway -ki ")) {
      swayPID.ki = command.substring(9).toFloat();
      Serial.print("Sway ki is updated to "); Serial.print(swayPID.ki); Serial.println(".");
    }
    else if (command.startsWith("sway -kd ")) {
      swayPID.kd = command.substring(9).toFloat();
      Serial.print("Sway kd updated to "); Serial.print(swayPID.kd); Serial.println(".");
    }
    else if (command.startsWith("sway -target ")) {
      targetSway = command.substring(13).toFloat();
      Serial.print("Target sway updated to "); Serial.print(targetSway); Serial.println(".");
    }

    // Testing Thrusters
    else if(command.startsWith("test ")) {
      int spaceIDX = command.indexOf(' ',5);
      if(spaceIDX>0) {
        String thrusterName = command.substring(5,spaceIDX);
        int power = command.substring(spaceIDX + 1).toInt();
        testThruster(thrusterName, power);
      } else {
        Serial.println("Invalid.");
      }
    }

    // Disabling Thrusters
    else if (command.startsWith("disable ")) {
      String thrusterName = command.substring(8);
      if(thrusterName.equalsIgnoreCase("FR")) {
      setThrusterStatus(FR, FR_ESC, false);
      Serial.print(FR.name); Serial.println(" disabled.");
      } 
      else if(thrusterName.equalsIgnoreCase("FL")) {
        setThrusterStatus(FL, FL_ESC, false);
        Serial.print(FL.name); Serial.println(" disabled.");
      } 
      else if(thrusterName.equalsIgnoreCase("BL")) {
        setThrusterStatus(BL, BL_ESC, false);
        Serial.print(BL.name); Serial.println(" disabled.");
      } 
      else if(thrusterName.equalsIgnoreCase("BR")) {
        setThrusterStatus(BR, BR_ESC, false);
        Serial.print(BR.name); Serial.println(" disabled.");
      } 
      else if(thrusterName.equalsIgnoreCase("VR")) {
        setThrusterStatus(VR, VR_ESC, false);
        Serial.print(VR.name); Serial.println(" disabled.");
      }
      else if(thrusterName.equalsIgnoreCase("VL")) {
        setThrusterStatus(VL, VL_ESC, false);
        Serial.print(VL.name); Serial.println(" disabled.");
      }
      else if (thrusterName.equalsIgnoreCase("-a")) {
        setALLThrustersStatus(false);
      }
      else if(thrusterName.equalsIgnoreCase("-v")) {
        setVerticalThrustersStatus(false);
      }
      else if(thrusterName.equalsIgnoreCase("-h")) {
        setHorizontalThrustersStatus(false);
      }
      else {
        Serial.print("Thruster "); Serial.print(thrusterName); Serial.println(" Not Found!");
      }
    }

    // Enabling Thrusters
    else if (command.startsWith("enable ")) {
      String thrusterName = command.substring(7);
      if(thrusterName.equalsIgnoreCase("FR")) {
      setThrusterStatus(FR, FR_ESC, true);
      Serial.print(FR.name); Serial.println(" enabled.");
      } 
      else if(thrusterName.equalsIgnoreCase("FL")) {
        setThrusterStatus(FL, FL_ESC, true);
        Serial.print(FL.name); Serial.println(" enabled.");
      } 
      else if(thrusterName.equalsIgnoreCase("BL")) {
        setThrusterStatus(BL, BL_ESC, true);
        Serial.print(BL.name); Serial.println(" enabled.");
      } 
      else if(thrusterName.equalsIgnoreCase("BR")) {
        setThrusterStatus(BR, BR_ESC, true);
        Serial.print(BR.name); Serial.println(" enabled.");
      } 
      else if(thrusterName.equalsIgnoreCase("VR")) {
        setThrusterStatus(VR, VR_ESC, true);
        Serial.print(VR.name); Serial.println(" enabled.");
      }
      else if(thrusterName.equalsIgnoreCase("VL")) {
        setThrusterStatus(VL, VL_ESC, true);
        Serial.print(VL.name); Serial.println(" enabled.");
      }
      else if(thrusterName.equalsIgnoreCase("-a")) {
        setALLThrustersStatus(true);
      }
      else if(thrusterName.equalsIgnoreCase("-v")) {
        setVerticalThrustersStatus(true);
      }
      else if(thrusterName.equalsIgnoreCase("-h")) {
        setHorizontalThrustersStatus(true);
      }
      else {
        Serial.print("Thruster "); Serial.print(thrusterName); Serial.println(" Not Found!");
      }
    }

    // Stopping Thrusters
    else if (command.startsWith("emergency") || command == "stop") {
      emergencyStop();
    }
    else if (command.startsWith("stop ")) {
      String thrusterName = command.substring(5);
      if(thrusterName.equalsIgnoreCase("FR")) {
        stopThruster(FR, FR_ESC);
        Serial.print(FR.name); Serial.println(" stopped.");
      } 
      else if(thrusterName.equalsIgnoreCase("FL")) {
        stopThruster(FL, FL_ESC);
        Serial.print(FL.name); Serial.println(" stopped.");
      } 
      else if(thrusterName.equalsIgnoreCase("BL")) {
        stopThruster(BL, BL_ESC);
        Serial.print(BL.name); Serial.println(" stopped.");
      } 
      else if(thrusterName.equalsIgnoreCase("BR")) {
        stopThruster(BR, BR_ESC);
        Serial.print(BR.name); Serial.println(" stopped.");
      } 
      else if(thrusterName.equalsIgnoreCase("VR")) {
        stopThruster(VR, VR_ESC);
        Serial.print(VR.name); Serial.println(" stopped.");
      }
      else if(thrusterName.equalsIgnoreCase("VL")) {
        stopThruster(VL, VL_ESC);
        Serial.print(VL.name); Serial.println(" stopped.");
      }
      else if(thrusterName.equalsIgnoreCase("-a")) {
        emergencyStop();
      }
      else if(thrusterName.equalsIgnoreCase("-v")) {
        stopVertical();
      }
      else if(thrusterName.equalsIgnoreCase("-h")) {
        stopHorizontal();
      }
      else {
        Serial.print("Thruster "); Serial.print(thrusterName); Serial.println(" Not Found!");
      }
    }

    // General Commands
    else if(command == "reset") {
      resetALL_PIDs();
    } else if (command.startsWith("reset ")) {
      String pidName = command.substring(7);
      if(pidName.equalsIgnoreCase("-heave")) {
        resetPID(heavePID);
        Serial.println("Heave PID has been reseted.");
      }
      else if (pidName.equalsIgnoreCase("-yaw")) {
        resetPID(yawPID);
        Serial.println("Yaw PID has been reseted.");
      }
      else if (pidName.equalsIgnoreCase("-surge")) {
        resetPID(surgePID);
        Serial.println("Surge PID has been reseted.");
      }
      else if (pidName.equalsIgnoreCase("-sway")) {
        resetPID(swayPID);
        Serial.println("Sway PID has been reseted.");
      }
      else if (pidName.equalsIgnoreCase("-a")) {
        resetALL_PIDs();
      }
      else {
        Serial.print("PID "); Serial.print(pidName); Serial.println(" Not Found!");
      }
    }


    else if (command.startsWith("debug ")) {
      String restOfCommand = command.substring(6);
      if(restOfCommand.equalsIgnoreCase("-dof")) {
        printDOFInfo();
      }
      else if (restOfCommand.equalsIgnoreCase("FR")) {
        printThrusterInfo(FR);
      }
      else if (restOfCommand.equalsIgnoreCase("FL")) {
        printThrusterInfo(FL);
      }
      else if (restOfCommand.equalsIgnoreCase("BR")) {
        printThrusterInfo(BR);
      }
      else if (restOfCommand.equalsIgnoreCase("BL")) {
        printThrusterInfo(BL);
      }
      else if (restOfCommand.equalsIgnoreCase("VR")) {
        printThrusterInfo(VR);
      }
      else if (restOfCommand.equalsIgnoreCase("VL")) {
        printThrusterInfo(VL);
      }
      else if (restOfCommand.equalsIgnoreCase("-v")) {
        printVerticalThrustersInfo();
      }
      else if (restOfCommand.equalsIgnoreCase("-h")) {
        printHorizontalThrustersInfo();
      }
      else if (restOfCommand.equalsIgnoreCase("-a")) {
        printALLThrustersInfo();
      }
    }

    else if (command == "help") {
      Serial.println("Available Commands:");
      
      Serial.println("\t\"heave\" -kp/-ki/-kd/target [value] -> Tune Heave PID.");
      Serial.println("\t\"yaw\" -kp/-ki/-kd/target [value] -> Tune Yaw PID.");
      Serial.println("\t\"surge\" -kp/-ki/-kd/target [value] -> Tune Surge PID.");
      Serial.println("\t\"sway\"-kp/-ki/-kd/target [value] -> Tune Sway PID.");
      Serial.println("\t\"reset\" -a/-heave/-yaw/-surge/-sway -> Reset ALL or Heave or Yaw or Sway or Surge respectively.");
      Serial.println("\t\"reset\" -> Resets ALL PIDs.");
      Serial.println();
      Serial.println("\t\"test [thruster_name] [power]\" -> Test a thruster.");
      Serial.println("\t\"enable/disable [thruster_name]\" -> Enable/Disable a thruster.");
      Serial.println("\t\"enable/disable -a/-v/-h\" -> Enables=/Disable ALL or Vertical or Horizontal respectively.");
      Serial.println("\t\"stop [thruster_name]\" -> Stop a thruster.");
      Serial.println("\t\"stop/emergency\" -> Stop ALL Thrusters.");
      Serial.println("\t\"stop -a/-v/-h\" -> Stop ALL or Vertical or Horizontal respectively.");
      Serial.println();
      Serial.println("\t\"debug [thruster_name]\" -> Shows Thruster info.");
      Serial.println("\t\"debug -dof/-v/-h/-a\" -> Shows DOF info or vertical thrusters info or horizontal thrusters info or all thrusters respectively.");
    }
  }
}