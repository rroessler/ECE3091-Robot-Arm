/*
*   Lab 2 - I
*
*   This .ino file is the code for determining IK and FK relationships between
*   our Robot Arm's joint angles and its current end effector position.
*/

// setting up storage variables needed
float jointAngles[4];
float endEffectorPos[3];

// constant variables throughout the code
const float armLengths[4] = {6, 8, 8, 6}; // setting constant robot arm lengths between joints

// initialization of Arduino program
void setup()
{
    // open serial port for debugging
    Serial.begin(9600);

    // calculate Forward Kinematics
    jointAngles[0] = 0;
    jointAngles[1] = 0;
    jointAngles[2] = 0;

    calc_FK(jointAngles);

    // calculate Inverse Kinematics
    endEffectorPos[0] = 0;
    endEffectorPos[1] = 0;
    endEffectorPos[2] = 0;

    calc_IK(endEffectorPos);
}

// Arduino loop program
void loop()
{

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
    jointAngles[1] = (atan2(r2, r1) - acos((sq(armLengths[2]) - sq(armLengths[1]) - sq(r3)) / (-2 * armLengths[1] * r3))) * RAD_TO_DEG;
    jointAngles[2] = (M_PI - acos((sq(r3) - sq(armLengths[1]) - sq(armLengths[2])) / (-2 * armLengths[1] * armLengths[2]))) * RAD_TO_DEG;
}