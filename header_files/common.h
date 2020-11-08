#ifndef _COMMON_H_
#define _COMMON_H_

#include <VarSpeedServo.h>

// global storage variables of current joint angles and end effector position
extern float jointAngles[4];
extern float endEffectorPos[3];

// setting constant variables need
const extern float angleOffsets[4];     // constant distances for robot arm sections
const extern bool debug;                // debug boolean for serial monitor

// current state defined by integer
extern int currentState;        // set to default of 0
extern int nextState;           // set a next state variable to hold the next state we go to

// setting up the servos given from VarSpeedServo.cpp
extern VarSpeedServo baseServo;
extern VarSpeedServo armServo;
extern VarSpeedServo wristServo;
extern VarSpeedServo gripperServo;

#endif