/*
*   Lab 2 - II.2
*
*
*/

#include <VarSpeedServo.h>

// setting up storage variables needed
float jointAngles[4];
float endEffectorPos[3];

// setting constant variables needed
const float armLengths[4] = {6, 8, 8, 6};    // setting constant robot arm lengths between joints
const float angleOffsets[4] = {0, 0, 180, 30}; // angle offset constants for joint angle calculations
const int debug = true;                      // debug boolean for serial monitor display

// setting up PIN constants
const int basePIN = 6;
const int armPIN = 7;
const int wristPIN = 8;
const int gripperPIN = 9;

// setting up the servos given from VarSpeedServo.cpp
VarSpeedServo baseServo;
VarSpeedServo armServo;
VarSpeedServo wristServo;
VarSpeedServo gripperServo;

// default speeds for servos as indexed
int defaultSpeed[4] = {30, 30, 30, 30};


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
void change_servoPos(int servoSpeed[4] = defaultSpeed)
{
    // iterate over each global joint angle
    for (int i = 0; i < sizeof(jointAngles) / sizeof(jointAngles[0]); i++)
    {
        // depending on which angle index we have switch to required servo
        switch (i)
        {
        case 0:
            baseServo.write(jointAngles[i] + angleOffsets[0], servoSpeed[i]);
            break;
        case 1:
            armServo.write(jointAngles[i] + angleOffsets[1], servoSpeed[i]);
            break;
        case 2:
            wristServo.write(jointAngles[i] + angleOffsets[2], servoSpeed[i]);
            break;
        case 3:
            gripperServo.write(jointAngles[i] + angleOffsets[3], servoSpeed[i]);
            break;
        }
    }

    // make program wait for servos to finish
    baseServo.wait();
    armServo.wait();
    wristServo.wait();
    gripperServo.wait();
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
void move_toAngles(float nextAngles[], bool disp = debug, int servoSpeed[4] = defaultSpeed)
{
    // determine next position
    if (disp)
        Serial.println("-- Calculating Next Position --");
    calc_FK(jointAngles);
    if (disp)
        print_currentPos();

    // and move to the next position with given speeds
    if (disp)
        Serial.println("-- Moving to Next Position --");
    change_servoPos(servoSpeed);

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
void move_toPos(float nextPos[], bool disp = debug, int servoSpeed[4] = defaultSpeed) {
    // determine next position joint angles
    if (debug) Serial.println("-- Calculating Next Position --");
    calc_IK(endEffectorPos);
    if (debug) print_currentAngles();

    // and move to the next position with given speeds
    if (debug) Serial.println("-- Moving to Next Position --");
    change_servoPos(servoSpeed);

    // and print current position
    if (debug) print_currentPos();
}

// initialization of Arduino program
void setup()
{
    // open serial port for debugging
    Serial.begin(9600);

    // initialise end effector and joint joint angles
    jointAngles[0] = 90;  // base rotation
    jointAngles[1] = 90;  // arm rotation
    jointAngles[2] = 0; // wrist rotation
    jointAngles[3] = 0;   // gripper rotation

    // attaching Arduino PINs and Servo Pins
    baseServo.attach(basePIN);
    armServo.attach(armPIN);
    wristServo.attach(wristPIN);
    gripperServo.attach(gripperPIN);

    // reseting arm to default position as given previously in setup()
    change_servoPos();

    // determine current cartesian coordinates
    calc_FK(jointAngles);

    // and print to serial monitor the current positioning
    if (debug)
    {
        print_currentPos();
        print_currentAngles();
    }
}

// Arduino main loop program
void loop()
{
    // read input initially
    Serial.println("-- Please enter x, y, z Coordinates --");
    while (!Serial.available()) {}
    endEffectorPos[0] = Serial.parseFloat();
    Serial.print("x: ");
    Serial.println(endEffectorPos[0]);
    delay(100);
    while (!Serial.available()) {}
    endEffectorPos[1] = Serial.parseFloat();
    Serial.print("y: ");
    Serial.println(endEffectorPos[1]);
    delay(100);
    while (!Serial.available()) {}
    endEffectorPos[2] = Serial.parseFloat();
    Serial.print("z: ");
    Serial.println(endEffectorPos[2]);

    // move to chosen location
    move_toPos(endEffectorPos);

    delay(1000);
}

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
    jointAngles[1] = 180 - (atan2(r2, r1) + acos((sq(armLengths[1]) + sq(r1) + sq(r2) - sq(armLengths[2]))/(2 * armLengths[1] * r3))) * RAD_TO_DEG;
    jointAngles[2] = -(90 - acos((sq(armLengths[1]) + sq(armLengths[2]) - sq(r1) - sq(r2)) / (2 * armLengths[1] * armLengths[2])) * RAD_TO_DEG);

    if (jointAngles[2] + angleOffsets[2] > 180) jointAngles[2] = 0;
}