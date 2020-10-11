#include <Arduino.h>
#include <common.h>

// #include <Kinematics.h>

// setting constant variables need
const float armLengths[4] = {6, 8, 8, 6};        // constant distances for robot arm sections
const float angleOffsets[4] = {0, 0, 180, 30};   // constant angle offsets for servos if needed

/*
*   FUNCTION:
*       Displays in the serial monitor the Current Position in Cartesian Coordinates
*
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil.
*/
void print_currentPos()
{
    Serial.println("-- Current Position --");
    Serial.print("x: ");
    Serial.println(endEffectorPos[0], 3);
    Serial.print("y: ");
    Serial.println(endEffectorPos[1], 3);
    Serial.print("z: ");
    Serial.println(endEffectorPos[2], 3);
}

/*
*   FUNCTION:
*       Displays in the serial monitor the Current Angles in Degrees
*
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil.
*/
void print_currentAngles()
{
    Serial.println("-- Current Angles --");
    Serial.print("Base: ");
    Serial.println(jointAngles[0] + angleOffsets[0], 3);
    Serial.print("Arm: ");
    Serial.println(jointAngles[1] + angleOffsets[1], 3);
    Serial.print("Wrist: ");
    Serial.println(jointAngles[2] + angleOffsets[2], 3);
}

/*
*   FUNCTION:
*       Calculates and saves the end effector position for given joint angle values
*
*   INPUT:
*       float jointAngles[] -> array of angles given as float data
*
*   OUPUT:
*       nil.
*       => however saves resulting end effector cartesian coordinates to global endEffectorPos[] array
*/
void calc_FK(float jointAngles[])
{
    float radius = armLengths[1] * cos(jointAngles[1] * DEG_TO_RAD) + armLengths[2] * cos((jointAngles[1] + jointAngles[2]) * DEG_TO_RAD) + armLengths[3];

    endEffectorPos[0] = radius * cos(jointAngles[0] * DEG_TO_RAD);
    endEffectorPos[1] = radius * sin(jointAngles[0] * DEG_TO_RAD);
    endEffectorPos[2] = armLengths[0] + armLengths[1] * sin(jointAngles[1] * DEG_TO_RAD) + armLengths[2] * sin(jointAngles[1] * DEG_TO_RAD + jointAngles[2] * DEG_TO_RAD);
}

/*
*   FUNCTION:
*       Calculates and saves the joint angles for given end effector cartesian coordinates
*
*   INPUT:
*       float endEffectorPos[] -> array of end effector values given as float data
*
*   OUPUT:
*       nil.
*       => however saves resulting joint angles calculated into global jointAngles[]
*/
void calc_IK(float endEffectorPos[])
{
    float r1 = sqrt(sq(endEffectorPos[0]) + sq(endEffectorPos[1])) - armLengths[3];
    float r2 = endEffectorPos[2] - armLengths[0];
    float r3 = sqrt(sq(r1) + sq(r2));

    jointAngles[0] = (atan2(endEffectorPos[1], endEffectorPos[0])) * RAD_TO_DEG;
    jointAngles[1] = 180 - (atan2(r2, r1) + acos((sq(armLengths[1]) + sq(r1) + sq(r2) - sq(armLengths[2])) / (2 * armLengths[1] * r3))) * RAD_TO_DEG;
    jointAngles[2] = -(90 - acos((sq(armLengths[1]) + sq(armLengths[2]) - sq(r1) - sq(r2)) / (2 * armLengths[1] * armLengths[2])) * RAD_TO_DEG);

    if (jointAngles[2] + angleOffsets[2] > 180)
        jointAngles[2] = 0;
}