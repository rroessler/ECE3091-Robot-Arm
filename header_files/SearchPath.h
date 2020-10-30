#ifndef _SEARCHPATH_H_
#define _SEARCHPATH_H_

// forward declaration to use functions where required
void update_search_angle_base(bool side = true); // when side == true (CUBES LEFT), side == false (CUBES RIGHT)

int search_path_max_states();
void update_search_angle_arm(int counter);       // based on counter value we can use switch states for precise movement

#endif