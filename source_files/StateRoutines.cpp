#include <VarSpeedServo.h>
#include <Arduino.h>

#include <Movement.h>
#include <Kinematics.h>
#include <common.h>

#include <StateRoutines.h>

float jointAngles[4];
float endEffectorPos[3];

// setting up constants required for the routines
const float defaultAngles[4] = {90, 90, 0, 0}; // default angles for robot reset/init
const bool debug = true;                       // boolean for whether to print serial statements
const int grabSpeed = 50;                      // gripper speed for picking up, faster = more reliable in actually moving

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

void update_state()
{
    if (currentState != nextState)
    {
        currentState = nextState;
    }
}

void initialise_robot()
{
    // initialise end effector and joint joint angles
    jointAngles[0] = defaultAngles[0]; // base rotation
    jointAngles[1] = defaultAngles[1]; // arm rotation
    jointAngles[2] = defaultAngles[2]; // wrist rotation
    jointAngles[3] = defaultAngles[3]; // gripper rotation

    calc_FK(jointAngles);

    // initialising the servos
    goto_start_pos();

    // and now set the next state
    nextState = 1;
}

void goto_start_pos()
{
    // attaching Arduino PINs and Servo Pins after writing initial positions
    // - this counteracts the problem of servos jumping to a distinct other "zero" position
    //   on start up

    // write position
    baseServo.write(jointAngles[0]);
    armServo.write(jointAngles[1]);
    wristServo.write(jointAngles[2]);
    gripperServo.write(jointAngles[3]);

    // quickly activate and deactivate power to servos to trigger false initial position
    baseServo.attach(basePIN);
    armServo.attach(armPIN);
    wristServo.attach(wristPIN);
    gripperServo.attach(gripperPIN);

    baseServo.detach();
    armServo.detach();
    wristServo.detach();
    gripperServo.detach();

    // now recall position to move to again
    int speed[4] = {20, 20, 20, 20};
    move_toPos(endEffectorPos, speed);

    // and attach to actually move to desired default position
    baseServo.attach(basePIN);
    armServo.attach(armPIN);
    wristServo.attach(wristPIN);
    gripperServo.attach(gripperPIN);
}

void run_search_path()
{
    // we note a search speed that should be as slow as possible
    // int searchSpeed[4] = {5, 5, 5, 5};
    int searchSpeed[4] = {30, 30, 30, 30};

    // for search path we need to start at a position and arc around the base.
    // this is best done through setting angles

    // we start by putting robot in start path position at given angles
    float startPoint[4] = {180, 180, 0, 30};
    move_toAngles(startPoint, searchSpeed); // we will wait for this to finish
    delay(2000);

    // now we just need to decrement to base position and move along the arc
    // we continue this until we have search the entire area
    int counter = 0;
    while (true)
    {
        // ping pong the base arc
        if (jointAngles[0] == 90)
        {
            jointAngles[0] = 180;
        }
        else
        {
            jointAngles[0] = 90;
        }
        move_toAngles(jointAngles, searchSpeed, false);
        delay(1000);

        // and move closer to robot on each arc
        jointAngles[1] -= 10;
        jointAngles[2] -= 15;
        move_toAngles(jointAngles, searchSpeed);
        delay(1000);

        if (counter == 4)
        {
            break;
        }

        counter++;
    }

    nextState = 9;
}

void start_pickup_routine()
{
    // found block and hopefully positioned well enough to now pick up a block

    // *** start by opening claw
    set_gripper_angle(60, 60, true);
    delay(2000);

    // *** then move down z axis to suitable position
    // couple ways this could be done, could alter current end effector position
    // however most accurate will be to set a decline angle for arm and wrist servos
    jointAngles[1] += 10;
    jointAngles[2] -= 10;

    calc_FK(jointAngles);
    int speed[4] = {20, 20, 20, 20};
    move_toAngles(jointAngles, speed);
    delay(1000);

    // *** close claw
    set_gripper_angle(0, 60, true);
    delay(1000);

    /* TEST BLOCK SIZE WITH CLAW ANGLE */

    // *** lift to set height

    // *** move to colour sensor
    nextState = 4; // next state is move to and do colour test
}

void handle_debug_input()
{
    // temporary function to handle serial inputs as desired
    // recreate for any tests you want, or better yet switch case them with a desired function in the case

    // move to input position
    float point[4];
    Serial.println("--- Please enter a starting x, y, z coordinate ---");
    record_point(point);

    // move to pos
    int speed[4] = {50, 50, 50, 50};
    move_toPos(point, speed);

    // ask if we want to test pickup routine
    int run = 0;
    Serial.println();
    Serial.println("--- Please enter 0 to choose another point, or 1 to start pickup ---");
    while (!Serial.available())
    {
    }
    run = Serial.parseInt();

    if (run == 1)
    {
        // conduct pickup routine
        nextState = 3;
    }
    else
    {
        // choose another point
        nextState = 10;
    }
}

// debug code for recording points
void record_point(float point[])
{
    while (!Serial.available())
    {
    }
    point[0] = Serial.parseFloat();
    Serial.print("x: ");
    Serial.println(point[0]);
    delay(100);
    while (!Serial.available())
    {
    }
    point[1] = Serial.parseFloat();
    Serial.print("y: ");
    Serial.println(point[1]);
    delay(100);
    while (!Serial.available())
    {
    }
    point[2] = Serial.parseFloat();
    Serial.print("z: ");
    Serial.println(point[2]);
    Serial.println();
}