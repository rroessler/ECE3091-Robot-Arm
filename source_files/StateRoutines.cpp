#include <VarSpeedServo.h>

#include <Kinematics.h>
#include <common.h>

#include <StateRoutines.h>

float jointAngles[4];
float endEffectorPos[3];

// setting up constants required for the routines
const float defaultAngles[4] = {90, 90, 0, 0};  // default angles for robot reset/init
const bool debug = true;                        // boolean for whether to print serial statements

// initialise current state here
int currentState = 0;
int nextState = -1;

// setting up PIN constants
const int basePIN = 6;
const int armPIN = 7;
const int wristPIN = 8;
const int gripperPIN = 9;

// setting up servo initialisation
VarSpeedServo baseServo;
VarSpeedServo armServo;
VarSpeedServo wristServo;
VarSpeedServo gripperServo;

void update_state() {
    if (currentState != nextState) {
        currentState = nextState;
    }
}

void initialise_robot() {
    // initialise end effector and joint joint angles
    jointAngles[0] = defaultAngles[0];  // base rotation
    jointAngles[1] = defaultAngles[1];  // arm rotation
    jointAngles[2] = defaultAngles[2]; // wrist rotation
    jointAngles[3] = defaultAngles[3];   // gripper rotation

    calc_FK(jointAngles);

    // attaching Arduino PINs and Servo Pins
    baseServo.attach(basePIN);
    armServo.attach(armPIN);
    wristServo.attach(wristPIN);
    gripperServo.attach(gripperPIN);

    // and print to serial monitor the current positioning
    if (debug)
    {
        print_currentPos();
        print_currentAngles();
    }

    // and now set the next state
    nextState = 1;
}