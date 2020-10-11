#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_

#include <common.h>

// setting up constants required for the routines
int defaultSpeed[4] = {30, 30, 30, 30};         // default speed for Servos (note cannot be declared const, but should NOT be changed)
const extern bool debug;                        // grabbing global declaration of debug boolean

// forward declaration to use functions where required

void change_servoPos(int servoSpeed[4] = defaultSpeed, bool waitForFinish = true);

void move_toAngles(float nextAngles[], bool disp = debug, int servoSpeed[4] = defaultSpeed);
void move_toPos(float nextPos[], bool waitForFinish = true, bool disp = debug, int servoSpeed[4] = defaultSpeed);
void move_toPos_stepped(float startPos[], float endPos[], int stepSize);

#endif