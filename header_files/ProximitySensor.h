#ifndef _PROXIMITYSENSOR_H_
#define _PROXIMITYSENSOR_H_

// forward declaration to use functions where required
int read_proximity_sensor(int proximityPIN);
int read_proximity_average(int proximityPIN, int readCount = 10);

void detect_blocks(int proximityPIN, int currentCounter, int currentAverage, int threshold);
bool detect_block_held(int proximityPIN, int staticAverage, int threshold)

#endif