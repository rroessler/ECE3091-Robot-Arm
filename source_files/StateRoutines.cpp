#include <VarSpeedServo.h>
#include <Arduino.h>

#include <Movement.h>
#include <Kinematics.h>
#include <ProximitySensor.h>
#include <SearchPath.h>
#include <ColourSensor.h>
#include <common.h>

#include <StateRoutines.h>

float jointAngles[4];
float endEffectorPos[3];

// setting up constants required for the routines
const float defaultAngles[4] = {90, 90, 0, 30}; // default angles for robot reset/init
const bool debug = true;                       // boolean for whether to print serial statements
const int grabSpeed = 50;                      // gripper speed for picking up, faster = more reliable in actually moving
static int proximityThreshold = 20;            // threshold value for proximity sensor
static bool foundSide = true;                  // found side of field for cubes

// initialise current state here
int currentState = 0;
int nextState = -1;

// setting up PIN constants
const int proximityPIN = 0;

const int basePIN = 6;
const int armPIN = 7;
const int wristPIN = 8;
const int gripperPIN = 9;

// setting up servo initialisation
VarSpeedServo baseServo;
VarSpeedServo armServo;
VarSpeedServo wristServo;
VarSpeedServo gripperServo;

// need some static values to hold current block knowledge
int blockSize = -1;   // 0 => small, 1 => large, -1 => unset
int blockColour = -1; // 0 => red, 1 => green, 2 => blue, -1 => unset
int blocksFound = 0;  // counter for number of blocks found

void update_state()
{
    if (currentState != nextState)
    {
        currentState = nextState;
    }
}

void initialise_colour_sensor() {
    // we first initialise the colour sensor pins, then get the sample white/black readings
    init_colour_sensor_pins();
    set_start_balance();

    delay(1000); // and quick delay before running full robot program
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
    init_start_pos();

    // and now set the next state
    nextState = 1;
}

void init_start_pos()
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

void goto_rest_pos() {
    jointAngles[0] = defaultAngles[0]; // base rotation
    jointAngles[1] = defaultAngles[1]; // arm rotation
    jointAngles[2] = defaultAngles[2]; // wrist rotation
    jointAngles[3] = defaultAngles[3]; // gripper rotation

    calc_FK(jointAngles);

    int speed[4] = {20, 20, 20, 20};
    move_toAngles(jointAngles, speed);
}

void run_search_path()
{
    // we note a search speed that should be as slow as possible
    int searchSpeed[4] = {3, 3, 3, 3};

    // for search path we need to start at a position and arc around the base.
    // this is best done through setting angles

    // we start by putting robot in start path position at given angles
    float startPoint[4] = {foundSide ? 180.0 : 0.0, 180, 0, 30};
    move_toAngles(startPoint, searchSpeed); // we will wait for this to finish

    // now we just need to decrement to base position and move along the arc
    // we continue this until we have search the entire area
    int counter = 0;
    while (true)
    {
        // now we want to take an average proximity reading
        int proximityAverage = read_proximity_average(proximityPIN);

        delay(1500);

        // ping pong the base arc
        update_search_angle_base(foundSide);

        // move the the base angle set (arc around search)
        move_toAngles(jointAngles, searchSpeed, false, false);

        // run block detect whilst servos are moving
        int detectCounter = 0;
        while (baseServo.isMoving() || wristServo.isMoving() || armServo.isMoving() || gripperServo.isMoving())
        {
            // detect for a block, this will update the detectCounter if a block is detected
            detectCounter = detect_blocks(proximityPIN, detectCounter, proximityAverage, proximityThreshold);
            delay(100);

            if (detectCounter >= 4)
            {
                // found a block, stop all servos and run pickup routine
                stop_servos();
                nextState = 3;
                return;
            }
        }
        delay(1000);

        // and move closer to robot on each arc
        update_search_angle_arm(counter);
        move_toAngles(jointAngles, searchSpeed, false, false);

        detectCounter = 0;
        while (baseServo.isMoving() || wristServo.isMoving() || armServo.isMoving() || gripperServo.isMoving())
        {
            // detect for a block, this will update the detectCounter if a block is detected
            detectCounter = detect_blocks(proximityPIN, detectCounter, proximityAverage, proximityThreshold);
            delay(100);

            if (detectCounter >= 4)
            {
                // found a block, stop all servos and run pickup routine
                stop_servos();
                nextState = 3;
                return;
            }
        }
        delay(1000);

        // set a max amount of times we can move inward
        if (counter == search_path_max_states())
        {
            break;
        }

        counter++;
    }

    // if we reach here we are at the end of the search path, so repeat for now
    nextState = 2;
}

void start_pickup_routine()
{
    // found block and hopefully positioned well enough to now pick up a block

    // *** start by opening claw
    set_gripper_angle(90, 60, true);
    delay(2000);

    // *** then move down z axis to suitable position
    // couple ways this could be done, could alter current end effector position
    // however most accurate will be to set a decline angle for arm and wrist servos
    jointAngles[2] -= 17;
    calc_FK(jointAngles);

    int speed[4] = {20, 20, 20, 20};
    move_toAngles(jointAngles, speed);
    delay(1000);

    /* TEST BLOCK SIZE WITH CLAW ANGLE */
    // to test block size we want to close to gripper angle that can hold the large block, but not the small

    //  start by taking an average proximity reading
    int proximityAverage = read_proximity_average(proximityPIN);
    set_gripper_angle(30, 60, true, false); // now grip the block
    delay(500);

    jointAngles[2] += 15;
    move_toAngles(jointAngles, speed); // and move up slightly

    // now we check if we have a small or large block
    if (detect_block_held(proximityPIN, proximityAverage, proximityThreshold)) {
        // this means we haven't picked up the block so we know it is small
        blockSize = 0; // set block size variable

        // need to pick up again
        jointAngles[2] += 15;
        move_toAngles(jointAngles, speed);

        delay(1000);

        // *** close claw properly
        set_gripper_angle(10, 60, true, false);
    } else {
        // otherwise we have found a large block
        blockSize = 1;
    }

    delay(1000);

    // *** lift to set height
    jointAngles[1] = 90;
    jointAngles[2] = 0;
    calc_FK(jointAngles);

    move_toAngles(jointAngles, speed);
    delay(1000);

    // *** set next state to colour determination
    nextState = 4; // next state is move to and do colour test
}

void move_to_colour_sensor() {
    // set a move speed
    int speed[4] = {20, 20, 20, 20};

    // move to the predetermined location of the colour sensor
    jointAngles[0] = 95;
    jointAngles[1] = 100;
    jointAngles[2] = -55;
    move_toAngles(jointAngles, speed);

    delay(2000);
}

void find_block_colour() {
    // run the block colour checker
    int colourFound = run_block_colour_multi_check(5);

    // now we have a value, we first alert if not 0, 1, 2
    if (colourFound == -1) {
        // set as lost state
        nextState = 10;
        return;
    }

    // otherwise we have a found colour so we set the colour found
    blockColour = colourFound;

    // now want to move to a higher position before placing in storage
    int speed[4] = {20, 20, 20, 20};
    jointAngles[0] = 90;
    jointAngles[1] = 100;
    jointAngles[2] = 0;
    calc_FK(jointAngles);
    move_toAngles(jointAngles, speed);
    delay(500);

    // and continue to storage placement state
    nextState = 5;
}

void place_block_in_storage() {
    // for testing right now we are just going to drop the block
    // and return back to the search routine
    set_gripper_angle(90, 60, true);
    delay(1000);

    // and go to desired routine depending on blocks found
    blocksFound += 1;

    if (blocksFound >= 6) {
        // finish
        nextState = 9;
    } else {
        // return to searching
        nextState = 6;
    }
}

void return_to_search() {
    // move to rest position before moving back to search routine
    goto_rest_pos();
    delay(1000);

    nextState = 2;
}

/* DEBUG FUNCTIONS */

void manual_change_state()
{
    Serial.println("--- Please enter a state to GOTO ---");
    while (!Serial.available())
    {
    }
    nextState = Serial.parseInt();

    delay(500);
}

void code_change_state(int stateNum)
{
    nextState = stateNum;
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
    Serial.println("--- Please enter 0 to choose another point, or integer for routine number ---");
    while (!Serial.available())
    {
    }
    run = Serial.parseInt();

    if (run != 0)
    {
        // conduct pickup routine
        nextState = run;
    }
    else
    {
        // choose another point
        nextState = 11;
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