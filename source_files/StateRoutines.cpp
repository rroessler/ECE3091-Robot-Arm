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
const bool debug = false;                        // boolean for whether to print serial statements
const int grabSpeed = 50;                       // gripper speed for picking up, faster = more reliable in actually moving
static int proximityThreshold = 20;             // threshold value for proximity sensor
static bool foundSide = false;                  // found side of field for cubes

// initialise current state here
int currentState = 0;
int nextState = -1;

// setting up PIN constants
const int proximityPIN = 0;    // Analogue INPUT
const int leverSwitchPIN = 12; // Digital INPUT

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


/*
*   FUNCTION:
*       Updates the currentState global value if the nextState variable
*       specified by another function here has changed.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil. Updates currentState with nextState if different.
*/
void update_state()
{
    if (currentState != nextState)
    {
        currentState = nextState;
    }
}


/*
*   FUNCTION:
*       Initialises the robot itself. To do so we set the robot to an initial position
*       and define any pin modes required.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil.
*/
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

    // initialize the block size lever switch as an INPUT
    pinMode(leverSwitchPIN, INPUT);
}


/*
*   FUNCTION:
*       Initialises the colour sensor by first initialising the colour sensor pins
*       then determing the colour balance of the room.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil. Changes next state.
*/
void initialise_colour_sensor()
{
    // we first initialise the colour sensor pins, then get the sample white/black readings
    init_colour_sensor_pins();
    set_start_balance();

    delay(1000); // and quick delay before running full robot program

    // and now set the next state
    nextState = 1;
}


/*
*   FUNCTION:
*       Attaches the Arduino Servo PINs in such a way that they don't move
*       to an internal default position of the VarSpeedServo library. If this 
*       function is not used, the servos default to bad positions and do so at 
*       full speed => could break the arm if not implemented.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil.
*/
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


/*
*   FUNCTION:
*       Moves the robot arm to the default rest position.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil.
*/
void goto_rest_pos()
{
    jointAngles[0] = defaultAngles[0]; // base rotation
    jointAngles[1] = defaultAngles[1]; // arm rotation
    jointAngles[2] = defaultAngles[2]; // wrist rotation
    jointAngles[3] = defaultAngles[3]; // gripper rotation

    calc_FK(jointAngles);

    int speed[4] = {20, 20, 20, 20};
    move_toAngles(jointAngles, speed);
}


/*
*   FUNCTION:
*       Conducts the search for the storage box. To do, the robot arm sweeps from
*       a base angle of 0 to 180, whilst continously checking for the storage box
*       edges with the proximity sensor. Upon finding the box, we determine the angle
*       we stopped at and use this to determine the side of the storage box.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil. Changes nextState and updates the found side the storage box is on.
*/
void storage_search_path()
{
    // we note a search speed that should be as slow as possible
    int searchSpeed[4] = {3, 3, 3, 3};

    // need to specify a starting position for storage box search and move to it
    float startPoint[] = {0, 158, 0, 30};
    move_toAngles(startPoint, searchSpeed);

    // now we need to take an average proximity sensor reading
    int proximityAverage = read_proximity_average(proximityPIN);
    delay(1000);

    // and move appropriately on base angle arc to find an edge of the storage box
    jointAngles[0] = 180;
    move_toAngles(jointAngles, searchSpeed, false, false);

    // check proximity sensor whilst base servo is moving
    int detectCounter = 0;
    int readAngle = -1;
    while (baseServo.isMoving())
    {
        detectCounter = detect_blocks(proximityPIN, detectCounter, proximityAverage, proximityThreshold);
        delay(100);

        if (detectCounter >= 4)
        {
            // got a valid hit of an edge so stop servos
            stop_servos();
            readAngle = baseServo.read(); // read the current base angle
            break;                        // and break out of while loop
        }
    }

    // determine side found!
    if (readAngle <= 90)
    {
        // less than 90 implies right side
        foundSide = true;
        Serial.println("Storage Side Found: Right");

        // display of side through RED LED
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(2, HIGH);
            delay(500);
            digitalWrite(2, LOW);
            delay(500);
        }
    }
    else
    {
        // greater than 90 implies left side
        foundSide = false;
        Serial.println("Storage Side Found: Left");

        // display of side through GREEN LED
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(3, HIGH);
            delay(500);
            digitalWrite(3, LOW);
            delay(500);
        }
    }
    delay(1000);

    // now we go to the rest position
    goto_rest_pos();

    // and now go to block searching mode
    nextState = 2;
}


/*
*   FUNCTION:
*       Conducts the block search path. We do so by ping ponging a sweep between 0 -> 65 or
*       180 -> 115 base angles (depending on storage box side) and moving the arm outwards
*       after each sweep. Before conducting a sweep, we take an average proximity sensor reading
*       to counteract shadow differences. Whilst sweeping, the proximity sensor is read and if a 
*       we detect a block based on number of detections we receive, we stop the servos and begin the pickup
*       routine.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil. Either repeats, or moves to the pickup routine.
*/
void blocks_search_path()
{
    // we note a search speed that should be as slow as possible
    int searchSpeed[4] = {3, 3, 3, 3};

    // for search path we need to start at a position and arc around the base.
    // this is best done through setting angles

    // we start by putting robot in start path position at given angles
    float startPoint[4] = {foundSide ? 180.0 : 0.0, 145, -48, 30};
    move_toAngles(startPoint, searchSpeed); // we will wait for this to finish

    // now we just need to decrement to base position and move along the arc
    // we continue this until we have search the entire area
    int counter = 0;
    while (true)
    {
        // now we want to take an average proximity reading
        int proximityAverage = read_proximity_average(proximityPIN);
        delay(1000);

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

        // set a max amount of times we can move outward
        if (counter == search_path_max_states())
        {   
            // break and causes robot to repeat the search path
            break;
        }

        counter++;
    }

    // if we reach here we are at the end of the search path, so repeat for now
    nextState = 2;
}


/*
*   FUNCTION:
*       Conducts the pickup routine. We first open the gripper claw, then move in such a way to
*       get in a good position to pickup the block. Begin the size detection by seeing when the
*       lever arm switch on the gripper is triggered. The block is then pick up and moved to a
*       rest state.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil. Picks up block and changes nextState.
*/
void start_pickup_routine()
{
    // found block and hopefully positioned well enough to now pick up a block

    // *** start by opening claw
    set_gripper_angle(75, 60, true);
    delay(1500);

    // *** then move down z axis to suitable position
    // couple ways this could be done, could alter current end effector position
    // however most accurate will be to set a decline angle for arm and wrist servos
    jointAngles[2] -= 20;

    int speed[4] = {20, 20, 20, 20};
    move_toAngles(jointAngles, speed);
    delay(1000);

    jointAngles[1] += 18;
    if (jointAngles[1] > 180)
    {
        jointAngles[1] = 180;
        jointAngles[2] -= 56;
    }
    calc_FK(jointAngles);

    move_toAngles(jointAngles, speed);
    delay(1000);

    // *** block size detection
    //  start by taking an average proximity reading
    int proximityAverage = read_proximity_average(proximityPIN);
    set_gripper_angle(5, 30, false, false); // now grip the block

    // see what angle the lever arm switch triggers at when the gripper is closed
    int leverHighVal = 0;
    while (gripperServo.isMoving())
    {
        // whilst the gripper servo is moving we want to read when lever switch goes to high
        if (digitalRead(leverSwitchPIN))
        {   
            // closed to we stop the servo (to counteract stalling) and read the value
            gripperServo.stop();
            leverHighVal = gripperServo.read();
            break;
        }
    }
    delay(2000);

    // determine block size based on when the lever value went HIGH
    blockSize = 1;
    Serial.println("Angle: " + String(leverHighVal));
    if (leverHighVal < 15)
    {  
        // SMALL block detected
        blockSize = 0;
        Serial.println("The Block is SMALL!");

        // display BLUE LED for small block
        for (int i = 0; i < 2; i++) {
            digitalWrite(4, HIGH);
            delay(500);
            digitalWrite(4, LOW);
            delay(500);
        }
    }
    else
    {
        // LARGE block detected
        // display ALL LEDs for large block
        Serial.println("The Block is LARGE!");
        for (int i = 0; i < 2; i++) {
            digitalWrite(2, HIGH);
            digitalWrite(3, HIGH);
            digitalWrite(4, HIGH);
            delay(500);
            digitalWrite(2, LOW);
            digitalWrite(3, LOW);
            digitalWrite(4, LOW);
            delay(500);
        }
    }
    Serial.println();
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


/*
*   FUNCTION:
*       Sets angles of servos and moves to the colour sensor position. Required
*       a seperate function to accurately move to the position
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil.
*/
void move_to_colour_sensor()
{
    // set a move speed
    int speed[4] = {20, 20, 20, 20};

    // move to the predetermined location of the colour sensor
    jointAngles[0] = 92;
    jointAngles[1] = 95;
    jointAngles[2] = -75;
    move_toAngles(jointAngles, speed);

    delay(2000);
}


/*
*   FUNCTION:
*       Conducts a block colour check. Does so by running the block colour check function
*       from the ColourSensor.h library. After getting a result displays the colour found
*       as an LED flash and moves to a high resting position before the next state.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil.
*/
void find_block_colour()
{
    // run the block colour checker
    int colourFound = run_block_colour_multi_check(5);

    Serial.println("Colour Found: " + String(colourFound));

    // now we have a value, we first alert if not 0, 1, 2
    if (colourFound == -1)
    {
        // set as lost state
        nextState = 10;
        return;
    }

    delay(1000);

    // display the colour found LED
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(colourFound + 2, HIGH);
        delay(500);
        digitalWrite(colourFound + 2, LOW);
        delay(500);
    }

    // otherwise we have a found colour so we set the colour found
    blockColour = colourFound;

    // now want to move to a higher position before placing in storage
    int speed[4] = {20, 20, 20, 20};
    jointAngles[0] = 90;
    jointAngles[1] = 70;
    jointAngles[2] = 0;
    calc_FK(jointAngles);
    move_toAngles(jointAngles, speed);
    delay(500);

    // and continue to storage placement state
    nextState = 5;
}


/*
*   FUNCTION:
*       Depending on the following items:
*           side -> side of storage box
*           blockColour -> integer for block colour
*           blockSize -> integer for block size
*       The following function programmatically moves to the correct placement
*       for a block in storage.
*
*       This function was made this way so that given the orientation of the box
*       never changes (alongside distance from middle point of the box to the arm),
*       this function could accurately place blocks as long as we have the base servo angle
*       for the center point of the box.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil. Changes nextState
*/
void place_block_in_storage()
{
    // determine block storage position

    // first choose arm and wrist position based on colour
    // does so by setting a middle value (eg: 90) and move left
    // or right on the base angle an amount to get to the correct 
    // colour storage compartment. As such is not hardcoded as per
    // a lookup table
    jointAngles[0] = 90;
    switch (blockColour)
    {
    case 0:
        // red
        jointAngles[0] += foundSide ? -15 : 42;
        break;
    case 1:
        // green
        jointAngles[0] += foundSide ? -28 : 28;
        break;
    case 2:
        // blue
        jointAngles[0] += foundSide ? -42 : 15;
        break;
    default:
        // unknown
        Serial.println("Error: Unknown Colour");
        break;
    }

    // and now choose base arc angle based on side and size
    // does so by setting servo angles to be close or further away. As such
    // this does not use hardcoded cartesian positions
    switch (blockSize)
    {
    case 0:
        // small
        jointAngles[1] = 135;
        jointAngles[2] = -20;
        break;
    case 1:
        // large
        jointAngles[1] = 154;
        jointAngles[2] = -3;
        break;
    default:
        // unknown
        Serial.println("Error: Unknown Size");
        break;
    }

    print_currentAngles();

    int speed[4] = {20, 20, 20, 20};
    move_toAngles(jointAngles, speed);
    delay(1000);

    // for testing right now we are just going to drop the block
    // and return back to the search routine
    gripperServo.attach(gripperPIN);
    set_gripper_angle(75, 60, true);
    delay(1000);

    // and go to desired routine depending on blocks found
    blocksFound += 1;
    blockColour = -1;
    blockSize = -1;

    if (blocksFound < -10)
    {
        // finish
        nextState = 9;
    }
    else
    {
        // return to searching
        nextState = 6;
    }
}


/*
*   FUNCTION:
*       Returns the robot to the search routine.
*   
*   INPUT:
*       nil.
*
*   OUPUT:
*       nil.
*/
void return_to_search()
{
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

void gripper_test()
{
    Serial.println("--- Enter any values to test the grip ---");
    while (!Serial.available())
    {
    }
    Serial.read();

    set_gripper_angle(100, 125, false);
    delay(1000);

    set_gripper_angle(0, 20, false, false);
    while (gripperServo.isMoving())
    {
        if (digitalRead(leverSwitchPIN))
        {
            gripperServo.stop();
            gripperServo.write(gripperServo.read());
            break;
        }
    }
    delay(1000);

    Serial.println("Gripper Angle: " + String(gripperServo.read()));
    delay(1000);

    set_gripper_angle(100, 125, false);
    delay(1000);

    nextState = 13;
}

void storage_placement_test()
{
    foundSide = false;
    int col[] = {0, 1, 2};
    int s[] = {0, 1};

    for (int x = 0; x < 3; x++)
    {
        for (int y = 0; y < 2; y++)
        {
            blockColour = col[x];
            blockSize = s[y];

            place_block_in_storage();

            jointAngles[1] = 90;
            jointAngles[2] = 0;
            int speed[] = {40, 40, 40, 40};
            move_toAngles(jointAngles, speed);

            goto_rest_pos();
            delay(1000);
        }
    }

    nextState = 9;
}