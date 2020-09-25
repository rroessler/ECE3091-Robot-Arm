/*
*   Lab 2 - II.3
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
void change_servoPos(int servoSpeed[4] = defaultSpeed, bool waitForFinish = true)
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
    if (waitForFinish) {
        baseServo.wait();
        armServo.wait();
        wristServo.wait();
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
void move_toPos(float nextPos[], bool waitForFinish = true, bool disp = debug, int servoSpeed[4] = defaultSpeed) {
    endEffectorPos[0] = nextPos[0];
    endEffectorPos[1] = nextPos[1];
    endEffectorPos[2] = nextPos[2];
    
    // determine next position joint angles
    if (debug) Serial.println("-- Calculating Next Position --");
    calc_IK(endEffectorPos);
    if (debug) print_currentAngles();

    // and move to the next position with given speeds
    if (debug) Serial.println("-- Moving to Next Position --");
    change_servoPos(servoSpeed, waitForFinish);

    // and print current position
    if (debug) print_currentPos();
}

void move_toPos_stepped(float startPos[], float endPos[], int stepSize) {
    if (stepSize == 0) {
        // step size of zero implies go straight to end position
        move_toPos(endPos);
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
    for (int i = 0; i < stepSize; i++) {
        // determine next point
        nextPoint[0] = startPos[0] + (i + 1) * ((magnitude) / (stepSize + 1)) * dir[0];
        nextPoint[1] = startPos[1] + (i + 1) * ((magnitude) / (stepSize + 1)) * dir[1];
        nextPoint[2] = startPos[2] + (i + 1) * ((magnitude) / (stepSize + 1)) * dir[2];
        
        move_toPos(nextPoint, false, true, servoSpeed);
    }
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
    Serial.println("--- Please enter a Path to Follow/Draw ---");
    Serial.println("=> Straight Line (type \"0\")");
    Serial.println("=> Square (type \"1\")");
    Serial.println("=> Arc (type \"2\")");
    Serial.println();
    while (!Serial.available()) {}
    int path = Serial.parseInt();

    switch (path) {
        case 0:
            follow_line();
            break;
        case 1:
            draw_box();
            break;
        case 2:
            draw_arc();
            break;
        default:
            Serial.println("You did not choose a valid Path!");
            Serial.println();
            break;
    }
}

void follow_line() {
    // set up start and end points
    float pointStart[3];
    float pointEnd[3];
    int stepSize;

    // save points as given
    Serial.println("--- Please enter a starting x, y, z coordinate ---");
    record_point(pointStart);
    Serial.println("--- Please enter an ending x, y, z coordinate ---");
    record_point(pointEnd);

    // specify a step size for => needed as otherwise arm travels with curvature between points
    Serial.println("--- Please enter a number of steps (more == straighter line, 0 == direct path) ---");
    while (!Serial.available()) {}
    stepSize = Serial.parseInt();
    Serial.print("Step Size: ");
    Serial.println(stepSize);
    Serial.println();

    delay(300);

    // initialise arm starting point
    Serial.println("--- Moving to start point ---");
    move_toPos(pointStart);

    // move arm from one point to another via steps
    delay(500);
    Serial.println("--- Moving along path desired! ---");
    Serial.println();

    move_toPos_stepped(pointStart, pointEnd, stepSize);

    Serial.println("--- Finished following path! ---");
    Serial.println();
    delay(500);
}

void record_point(float point[]) {
    while (!Serial.available()) {}
    point[0] = Serial.parseFloat();
    Serial.print("x: ");
    Serial.println(point[0]);
    delay(100);
    while (!Serial.available()) {}
    point[1] = Serial.parseFloat();
    Serial.print("y: ");
    Serial.println(point[1]);
    delay(100);
    while (!Serial.available()) {}
    point[2] = Serial.parseFloat();
    Serial.print("z: ");
    Serial.println(point[2]);
    Serial.println();
}

void draw_box() {
    // points to follow for a box
    float pointOne[3] = {-5, 14, 6};
    float pointTwo[3] = {5, 14, 6};
    float pointThree[3] = {5, 9, 6};
    float pointFour[3] = {-5, 9, 6};

    // move to starting position and away command to draw box
    move_toPos(pointOne);
    delay(300);
    
    Serial.println("--- Type 1 to begin drawing square, 0 to cancel ---");
    Serial.println();
    while (!Serial.available()) {}
    int choice = Serial.parseInt();

    if (choice) {
        move_toPos(pointTwo);
        move_toPos(pointThree);
        move_toPos(pointFour);
        move_toPos(pointOne);
        Serial.println("--- A square has been drawn! ---");
        Serial.println();
    } else {
        return;
    }
}

void draw_arc() {
    // need to first determine the number of points in our arc we want
    Serial.println("--- Please specify the number of points in your arc ---");
    while (!Serial.available()) {}
    int numPoints = Serial.parseInt();
    Serial.print("Number of Arc Points: ");
    Serial.println(numPoints);
    Serial.println();
    delay (100);

    float points[numPoints][3];

    // now need to record the required number of points
    for (int i = 0; i < numPoints; i++) {
        Serial.println("--- Please enter the x, y, z coords of Point " + String(i + 1) + "---");
        record_point(points[i]);
        delay(100);
    }

    Serial.println("--- Type 1 to begin drawing arc, 0 to cancel ---");
    Serial.println();
    while (!Serial.available()) {}
    int choice = Serial.parseInt();

    if (choice) {
        // once have our points just need to move to each one
        for (int i = 0; i < numPoints; i++) {
            move_toPos(points[i]);
        }
        Serial.println("--- Your arc has been drawn! ---");
        Serial.println();
    } else {
        return;
    }   
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