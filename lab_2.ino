#include <Servo.h>

float jointAngles[4];
float endEffectorPos[3];
const float armLengths[4] = {6, 8, 8, 6};
const float angleOffsets[4] = {0, 0, 0, 10};

// setting up PIN constants
const int basePIN = 6;
const int armPIN = 7;
const int wristPIN = 8;
const int gripperPIN = 9;

const int statusLED_PIN = 2;

// setting up the servos
Servo baseServo;
Servo armServo;
Servo wristServo;
Servo gripperServo;

void setup()
{
    Serial.begin(9600);

    // initialise end effector and joint joint angles

    // Calc Forward Kinematics
    jointAngles[0] = 90 + angleOffsets[0]; // base rotation
    jointAngles[1] = 90 + angleOffsets[1]; // arm rotation
    jointAngles[2] = 180 + angleOffsets[2]; // wrist rotation
    jointAngles[3] = 0 + angleOffsets[3]; // gripper rotation

    // Calc Inverse Kinematics
    // endEffectorPos[0] = 0; // end effector x
    // endEffectorPos[1] = 0; // end effector y
    // endEffectorPos[2] = 0; // end effector z

    // attaching PINs and initilising servos
    pinMode(statusLED_PIN, OUTPUT);
    
    baseServo.attach(basePIN);
    armServo.attach(armPIN);
    wristServo.attach(wristPIN);
    gripperServo.attach(gripperPIN);

    change_servoPos();
    calc_FK(jointAngles); // do initial calc of FK to get starting position
    print_currentPos();
}

void loop()
{
    digitalWrite(statusLED_PIN, HIGH);
    delay(1000);
    digitalWrite(statusLED_PIN, LOW);
    delay(100);
    digitalWrite(statusLED_PIN, HIGH);
    delay(100);
    digitalWrite(statusLED_PIN, LOW);
    delay(100);
}

void change_servoPos()
{
    for (int i = 0; i < sizeof(jointAngles) / sizeof(jointAngles[0]); i++)
    {
        switch (i)
        {
        case 0:
            baseServo.write(jointAngles[i]);
            break;
        case 1:
            armServo.write(jointAngles[i]);
            break;
        case 2:
            wristServo.write(jointAngles[i]);
            break;
        case 3:
            gripperServo.write(jointAngles[i]);
            break;
        }
    }
}

void print_currentPos() {
  Serial.print("x: ");
  Serial.println(endEffectorPos[0], 3);
  Serial.print("y: ");
  Serial.println(endEffectorPos[1], 3);
  Serial.print("z: ");
  Serial.println(endEffectorPos[2], 3);
}

void calc_FK(float jointAngles[])
{
    float radius = armLengths[1] * cos(jointAngles[1] * DEG_TO_RAD) + armLengths[2] * cos((jointAngles[1] + jointAngles[2]) * DEG_TO_RAD) + armLengths[3];

    endEffectorPos[0] = radius * cos(jointAngles[0] * DEG_TO_RAD);
    endEffectorPos[1] = radius * sin(jointAngles[0] * DEG_TO_RAD);
    endEffectorPos[2] = armLengths[0] + armLengths[1] * sin(jointAngles[1] * DEG_TO_RAD) + armLengths[2] * sin(jointAngles[1] * DEG_TO_RAD + jointAngles[2] * DEG_TO_RAD);
}

void calc_IK(float endEffectorPos[])
{
    float r1 = sqrt(sq(endEffectorPos[0]) + sq(endEffectorPos[1])) - armLengths[3];
    float r2 = endEffectorPos[2] - armLengths[0];
    float r3 = sqrt(sq(r1) + sq(r2));

    jointAngles[0] = (atan2(endEffectorPos[1], endEffectorPos[0])) * RAD_TO_DEG + angleOffsets[0];
    jointAngles[1] = (atan2(r2, r1) - acos((sq(armLengths[2]) - sq(armLengths[1]) - sq(r3)) / (-2 * armLengths[1] * r3))) * RAD_TO_DEG + angleOffsets[1];
    jointAngles[2] = (M_PI - acos((sq(r3) - sq(armLengths[1]) - sq(armLengths[2])) / (-2 * armLengths[1] * armLengths[2]))) * RAD_TO_DEG + angleOffsets[2];
}