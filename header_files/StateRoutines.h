#ifndef _STATEROUTINES_H_
#define _STATEROUTINES_H_

extern float jointAngles[4];
extern float endEffectorPos[3];

// initialise current state here
extern int currentState;

// forward declaration to use functions where required

void update_state();

void initialise_robot();
void goto_start_pos();

void run_search_path();

void start_pickup_routine();





void handle_debug_input();
void record_point(float point[]);


#endif