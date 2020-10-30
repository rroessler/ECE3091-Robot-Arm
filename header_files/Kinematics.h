#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

// forward declaration to use functions where required

void print_currentPos();
void print_currentAngles();

void calc_IK(float endEffectorPos[]);
void calc_FK(float jointAngles[]);

void set_angles_threshold();

#endif