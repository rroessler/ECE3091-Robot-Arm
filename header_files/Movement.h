#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_

#include <common.h>

// setting up constants required for the routines
const extern bool debug; // grabbing global declaration of debug boolean

// forward declaration to use functions where required

void change_servoPos(int servoSpeed[4], bool defaultSpeed = true, bool waitForFinish = true);
void stop_servos();
void set_gripper_angle(float angle, int servoSpeed, bool defaultSpeed = true, bool waitForFinish = true);

void move_toAngles(float nextAngles[], int servoSpeed[4], bool defaultSpeed = true, bool waitForFinish = true, bool disp = debug);
void move_toPos(float nextPos[], int servoSpeed[4], bool defaultSpeed = true, bool waitForFinish = true, bool disp = debug);
void move_toPos_stepped(float startPos[], float endPos[], int stepSize);

#endif