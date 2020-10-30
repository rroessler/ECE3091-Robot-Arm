#ifndef _COLOURSENSOR_H_
#define _COLOURSENSOR_H_

// forward declaration to use functions where required
void init_colour_sensor_pins();

void read_colour_sensor(int times);
void set_start_balance();
void check_colour(int type = -1);

int run_block_colour_multi_check(int times = 5);
int determine_block_colour();

#endif