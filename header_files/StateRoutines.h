#ifndef _STATEROUTINES_H_
#define _STATEROUTINES_H_

extern float jointAngles[4];
extern float endEffectorPos[3];

// initialise current state here
extern int currentState;

// forward declaration to use functions where required

void update_state();

void initialise_colour_sensor();
void initialise_robot();
void init_start_pos();

void storage_search_path();

void goto_rest_pos();
void blocks_search_path();
void return_to_search();

void start_pickup_routine();

void move_to_colour_sensor();
void find_block_colour();

void place_block_in_storage();




void manual_change_state();
void code_change_state(int stateNum);

void handle_debug_input();
void record_point(float point[]);

void gripper_test();
void storage_placement_test();


#endif