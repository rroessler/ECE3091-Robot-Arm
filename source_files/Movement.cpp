#include <Arduino.h>
#include <Kinematics.h>
#include <common.h>

#include <Movement.h>

// // setting up constants required for the routines
int defaultServoSpeed[4] = {20, 20, 20, 20}; // default speed for Servos (note cannot be declared const, but should NOT be changed)
// const bool debug = true;                        // boolean for whether to print serial statements

/*
*   FUNCTION:
*       Moves the servos to current joint angles given by jointAngles[]. Does
*       so at given speed else default speed.
*   
*   INPUT:
*       Speed for each servo to go.
*
*   OUPUT:
*       nil.
*/
void change_servoPos(int servoSpeed[4], bool defaultSpeed, bool waitForFinish)
{
    // iterate over each global joint angle
    for (int i = 0; i < sizeof(jointAngles) / sizeof(jointAngles[0]); i++)
    {
        // depending on which angle index we have switch to required servo
        switch (i)
        {
        case 0:
            baseServo.write(jointAngles[i] + angleOffsets[0], defaultSpeed ? defaultServoSpeed[i] : servoSpeed[i]);
            break;
        case 1:
            armServo.write(jointAngles[i] + angleOffsets[1], defaultSpeed ? defaultServoSpeed[i] : servoSpeed[i]);
            break;
        case 2:
            wristServo.write(jointAngles[i] + angleOffsets[2], defaultSpeed ? defaultServoSpeed[i] : servoSpeed[i]);
            break;
        case 3:
            gripperServo.write(jointAngles[i] + angleOffsets[3], servoSpeed[i]);
            break;
        }
    }

    // make program wait for servos to finish
    if (waitForFinish)
    {
        baseServo.wait();
        armServo.wait();
        wristServo.wait();
        gripperServo.wait();
    }
}

void set_gripper_angle(float angle, int servoSpeed, bool defaultSpeed, bool waitForFinish)
{
    // redefine global gripper angle;
    jointAngles[3] = angle;

    // and write to servo
    gripperServo.write(jointAngles[3] + angleOffsets[3], defaultSpeed ? defaultServoSpeed[4] : servoSpeed);

    // and wait for servo to finish if desired
    if (waitForFinish)
    {
        gripperServo.wait();
    }
}

/*
*   FUNCTION:
*       Moves the arm to given array of angles. Does so by first updating the new position,
*       then changes to servos position as required. And lastly prints results if desired.
*
*   INPUT:
*       float nextAngles[] -> angles to move servo to
*       bool disp = debug -> boolean for display in serial monitor
*       int servoSpeed[4] = defaultSpeed -> speed at which servos will move
*
*   OUTPUT:
*       nil.
*/
void move_toAngles(float nextAngles[], int servoSpeed[4], bool defaultSpeed, bool waitForFinish, bool disp)
{
    jointAngles[0] = nextAngles[0];
    jointAngles[1] = nextAngles[1];
    jointAngles[2] = nextAngles[2];
    jointAngles[3] = nextAngles[3];

    // determine next position
    if (disp)
        Serial.println("-- Calculating Next Position --");
    calc_FK(jointAngles);
    if (disp)
        print_currentPos();

    // and move to the next position with given speeds
    if (disp)
        Serial.println("-- Moving to Next Position --");
    change_servoPos(defaultSpeed ? defaultServoSpeed : servoSpeed, defaultSpeed, waitForFinish);

    // and print current angles
    if (disp)
        print_currentAngles();
}

/*
*   FUNCTION:
*       Moves the arm to given array of cartesian points. Does so by first updating the 
*       new position joint angles, then changes to servos position as required. And 
*       lastly prints results if desired.
*
*   INPUT:
*       float nextPos[] -> end effector positions to move to
*       bool disp = debug -> boolean for display in serial monitor
*       int servoSpeed[4] = defaultSpeed -> speed at which servos will move
*
*   OUTPUT:
*       nil.
*/
void move_toPos(float nextPos[], int servoSpeed[4], bool defaultSpeed, bool waitForFinish, bool disp)
{
    endEffectorPos[0] = nextPos[0];
    endEffectorPos[1] = nextPos[1];
    endEffectorPos[2] = nextPos[2];

    // determine next position joint angles
    if (debug)
        Serial.println("-- Calculating Next Position --");
    calc_IK(endEffectorPos);
    if (debug)
        print_currentAngles();

    // and move to the next position with given speeds
    if (debug)
        Serial.println("-- Moving to Next Position --");
    change_servoPos(defaultSpeed ? defaultServoSpeed : servoSpeed, defaultSpeed, waitForFinish);

    // and print current position
    if (debug)
        print_currentPos();
}

/*
*   FUNCTION:
*       Moves the arm to given array of cartesian points in steps. Does so by 
*       finding the direction from the start position to end position And
*       equating the concurrent step positions based on the steps given.
*
*   INPUT:
*       float startPos[] -> start position
*       float endPos[] -> end position
*       int stepSize -> number of steps to take from start to finish positions
*
*   OUTPUT:
*       nil.
*/
void move_toPos_stepped(float startPos[], float endPos[], int stepSize)
{
    if (stepSize == 0)
    {
        // step size of zero implies go straight to end position
        move_toPos(endPos, defaultServoSpeed);
        return;
    }

    // determine direction between start and end position
    float dir[3];
    dir[0] = endPos[0] - startPos[0];
    dir[1] = endPos[1] - startPos[1];
    dir[2] = endPos[2] - startPos[2];

    // normalize the direction vector
    float magnitude = sqrt(sq(dir[0]) + sq(dir[1]) + sq(dir[2]));
    dir[0] = dir[0] / magnitude;
    dir[1] = dir[1] / magnitude;
    dir[2] = dir[2] / magnitude;

    float nextPoint[3];
    int servoSpeed[4] = {75, 75, 75, 75};

    // iterate over a loop from 0 to stepSize - 1 and move to chosen points
    for (int i = 0; i < stepSize; i++)
    {
        // determine next point
        nextPoint[0] = startPos[0] + (i + 1) * ((magnitude) / (stepSize + 1)) * dir[0];
        nextPoint[1] = startPos[1] + (i + 1) * ((magnitude) / (stepSize + 1)) * dir[1];
        nextPoint[2] = startPos[2] + (i + 1) * ((magnitude) / (stepSize + 1)) * dir[2];

        move_toPos(nextPoint, servoSpeed, false);
    }
}